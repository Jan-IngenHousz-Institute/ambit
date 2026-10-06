#include "u_mlx.h"

Adafruit_I2CDevice *mlx = NULL;
static const char* TAG = "MLX";
double mlx_emissivity = 1.0;








// Melexis example constants: placeholders only. Nothing is computed from them —
// mlx_measure() reports MLX_TEMP_INVALID until mlx_init() has replaced them with this
// sensor's verified EEPROM values (mlx_calibration_ok).
static bool mlx_calibration_ok = false;
int32_t mlx_cali_PR = 0x00587f5b;
int32_t mlx_cali_PG = 0x04a10289;
int32_t mlx_cali_PT = 0xfff966f8;
int32_t mlx_cali_PO = 0x00001e0f;
int32_t mlx_cali_Ea = 4859535;
int32_t mlx_cali_Eb = 5686508;
int32_t mlx_cali_Fa = 53855361;
int32_t mlx_cali_Fb = 42874149;
int32_t mlx_cali_Ga = -14556410;
int16_t mlx_cali_Ha = 16384;
int16_t mlx_cali_Hb = 0;
int16_t mlx_cali_Gb = 9728;
int16_t mlx_cali_Ka = 10752;

// Field evidence for `mlxee`: real (not injected) failed transactions since boot, and how
// many attempts the last mlx_init() needed (0 = never succeeded).
static uint32_t mlx_bus_errors = 0;
static uint8_t mlx_calibration_attempts = 0;

#ifdef AMBIT_DIAG_MLX
// Bench fault injection (`mlxfail,<n>`): the next <n> register reads fail as a NACK would.
static uint16_t g_mlx_fail_reads = 0;
void mlx_diag_fail_reads(uint16_t n){ g_mlx_fail_reads = n; }
uint16_t mlx_diag_fail_reads_left(void){ return g_mlx_fail_reads; }
#endif

// Bus callbacks for the vendor driver (mlx_dev.cpp). Each reports its bus result:
// mlx_dev.cpp already stops on `ret < 0`, but these used to return 0 unconditionally,
// so a NACK or short read surfaced as a silent zero word — a wrong calibration constant
// at boot, or a DATA_RDY poll that could only end in its 2 s timeout.
int32_t mlx90632_i2c_read(int16_t register_address, uint16_t *value){
    uint8_t addr[2] = {(uint8_t)(register_address >> 8), (uint8_t)(register_address & 0x00FF)};
    uint8_t read_buff[2] = {0, 0};
#ifdef AMBIT_DIAG_MLX
    if (g_mlx_fail_reads){ g_mlx_fail_reads--; return -EIO; }
#endif
    if (mlx == NULL || !mlx->write_then_read(addr, 2, read_buff, 2, false)){
      mlx_bus_errors++;
      return -EIO;
    }
    *value = (uint16_t)((read_buff[0] << 8) | read_buff[1]);
    return 0;
  }

  int32_t mlx90632_i2c_write(int16_t register_address, uint16_t value){
    uint8_t buf[4] = {(uint8_t)(register_address >> 8), (uint8_t)(register_address & 0x00FF),
                      (uint8_t)(value >> 8), (uint8_t)(value & 0x00FF)};
    if (mlx == NULL || !mlx->write(buf, 4)){
      mlx_bus_errors++;
      return -EIO;
    }
    return 0;
  }

// ── Calibration constants ─────────────────────────────────────────────────────────
// Read once per boot, and everything the sensor reports depends on them. So: every word
// is its own checked 16-bit transaction — a 32-bit constant is LSW at its address, MSW at
// address + 1 (Melexis reference driver) — and the whole set is read twice and kept only
// if both passes agree word for word. The old path was one unchecked pass of 4-byte burst
// reads; on 2026-10-06 one boot in ~12 left AD85 reporting a 21 °C leaf as 317 °C until
// the next reboot (plans/MLX_CALIBRATION.md).
struct mlx_calibration_t {
  int32_t PR, PG, PT, PO, Ea, Eb, Fa, Fb, Ga;
  int16_t Gb, Ka, Ha, Hb;
};
static constexpr uint8_t kMlxCalibrationAttempts = 5;

static bool mlx_ee_read16(uint16_t address, int16_t* value){
  uint16_t word = 0;
  if (mlx90632_i2c_read(address, &word) < 0) return false;
  *value = (int16_t)word;
  return true;
}

static bool mlx_ee_read32(uint16_t address, int32_t* value){
  uint16_t lsw = 0, msw = 0;
  if (mlx90632_i2c_read(address, &lsw) < 0 || mlx90632_i2c_read(address + 1, &msw) < 0) return false;
  *value = (int32_t)(((uint32_t)msw << 16) | lsw);
  return true;
}

static bool mlx_read_calibration(mlx_calibration_t* c){
  return mlx_ee_read32(MLX90632_EE_P_R, &c->PR) && mlx_ee_read32(MLX90632_EE_P_G, &c->PG) &&
         mlx_ee_read32(MLX90632_EE_P_T, &c->PT) && mlx_ee_read32(MLX90632_EE_P_O, &c->PO) &&
         mlx_ee_read32(MLX90632_EE_Ea, &c->Ea) && mlx_ee_read32(MLX90632_EE_Eb, &c->Eb) &&
         mlx_ee_read32(MLX90632_EE_Fa, &c->Fa) && mlx_ee_read32(MLX90632_EE_Fb, &c->Fb) &&
         mlx_ee_read32(MLX90632_EE_Ga, &c->Ga) && mlx_ee_read16(MLX90632_EE_Gb, &c->Gb) &&
         mlx_ee_read16(MLX90632_EE_Ka, &c->Ka) && mlx_ee_read16(MLX90632_EE_Ha, &c->Ha) &&
         mlx_ee_read16(MLX90632_EE_Hb, &c->Hb);
}

// The compensation divides by P_G, Ea, Fa and Ha: zero there is a failed read, never a
// calibration. Deliberately no range checks — units differ, and a wrong bound would
// disable a good sensor for good.
static bool mlx_calibration_usable(const mlx_calibration_t* c){
  return c->PG != 0 && c->Ea != 0 && c->Fa != 0 && c->Ha != 0;
}

// Two checked reads of one word that must agree.
static bool mlx_read_word_twice(uint16_t address, uint16_t* value){
  uint16_t v1 = 0, v2 = 0;
  if (mlx90632_i2c_read(address, &v1) < 0 || mlx90632_i2c_read(address, &v2) < 0 || v1 != v2) return false;
  *value = v1;
  return true;
}

// The refresh rate lives in EEPROM (EE_MEDICAL_MEAS1/2), and mlx90632_set_refresh_rate()
// erases and rewrites each word from the value it has just read: an unverified read can
// write garbage into the sensor for good. Run it only when both words read back the same
// twice and actually differ in the refresh bits (a sensor already at the rate — every
// boot after the first — is never written), then confirm the result.
static void mlx_set_refresh_rate_verified(mlx90632_meas_t rate){
  uint16_t meas1 = 0, meas2 = 0;
  if (!mlx_read_word_twice(MLX90632_EE_MEDICAL_MEAS1, &meas1) ||
      !mlx_read_word_twice(MLX90632_EE_MEDICAL_MEAS2, &meas2)){
    ESP_LOGE(TAG, "MLX refresh rate not verifiable, EEPROM left untouched");
    return;
  }
  const uint16_t want = MLX90632_REFRESH_RATE_STATUS(rate);
  if (MLX90632_EE_REFRESH_RATE(meas1) == want && MLX90632_EE_REFRESH_RATE(meas2) == want) return;
  if (mlx90632_set_refresh_rate(rate) < 0 ||
      !mlx_read_word_twice(MLX90632_EE_MEDICAL_MEAS1, &meas1) ||
      !mlx_read_word_twice(MLX90632_EE_MEDICAL_MEAS2, &meas2) ||
      MLX90632_EE_REFRESH_RATE(meas1) != want || MLX90632_EE_REFRESH_RATE(meas2) != want){
    ESP_LOGE(TAG, "MLX refresh rate write not confirmed");
  }
}

  
void usleep(int min_range, int max_range){
  delayMicroseconds(min_range);
}

void msleep(int msecs){
  delay(msecs);
}



bool mlx_init(void){
    ESP_LOGV(TAG, "MLX90632 initate");
    mlx_calibration_ok = false;
    mlx_calibration_attempts = 0;
    // Allocated once: init runs again from the self-test and from mlx_ready().
    if (mlx == NULL) mlx = new Adafruit_I2CDevice(ADDR_I2C_MLX90632);
    if (!mlx->begin()){
        ESP_LOGE(TAG, "MLX device not found on I2C bus");
        return false;
    }

    // Each attempt repeats the version check too: with checked transactions a transient
    // NACK there is an error to retry, not a reason to give up for the session.
    mlx_calibration_t a, b;
    for (uint8_t attempt = 0; attempt < kMlxCalibrationAttempts && !mlx_calibration_ok; attempt++){
      memset(&a, 0, sizeof(a));
      memset(&b, 0, sizeof(b));
      if (mlx90632_init() >= 0 && mlx_read_calibration(&a) && mlx_read_calibration(&b) &&
          memcmp(&a, &b, sizeof(a)) == 0 && mlx_calibration_usable(&a)){
        mlx_cali_PR = a.PR; mlx_cali_PG = a.PG; mlx_cali_PT = a.PT; mlx_cali_PO = a.PO;
        mlx_cali_Ea = a.Ea; mlx_cali_Eb = a.Eb; mlx_cali_Fa = a.Fa; mlx_cali_Fb = a.Fb;
        mlx_cali_Ga = a.Ga; mlx_cali_Gb = a.Gb; mlx_cali_Ka = a.Ka; mlx_cali_Ha = a.Ha;
        mlx_cali_Hb = a.Hb;
        mlx_calibration_ok = true;
        mlx_calibration_attempts = attempt + 1;
      }else{
        delay(10);
      }
    }
    if (!mlx_calibration_ok){
      ESP_LOGE(TAG, "MLX calibration not verified after %u attempts", kMlxCalibrationAttempts);
      return false;
    }
    mlx_set_refresh_rate_verified(MLX90632_MEAS_HZ_16);

    mlx90632_set_emissivity(mlx_emissivity);

    ESP_LOGI(TAG, "MLX90632 initate OK");

    return true;

}

bool mlx_calibration_valid(void){
  return mlx_calibration_ok;
}

// A failed boot read heals on the next measurement instead of lasting until a reboot.
static bool mlx_ready(void){
  return mlx_calibration_ok || mlx_init();
}

double mlx_measure(double* object, double* ambient){

    int32_t ret = 0; /**< Variable will store return values */
    // double ambient; /**< Ambient temperature in degrees Celsius */
    // double object; /**< Object temperature in degrees Celsius */
    int16_t ambient_new_raw = 120;
    int16_t ambient_old_raw = 320;
    int16_t object_new_raw = 3210;
    int16_t object_old_raw = 1230;

    *object = MLX_TEMP_INVALID;
    *ambient = MLX_TEMP_INVALID;
    if (!mlx_ready()) return MLX_TEMP_INVALID;
    // On a bus error or DATA_RDY timeout the raw words above are placeholders: report
    // invalid rather than a temperature computed from them.
    ret = mlx90632_read_temp_raw(&ambient_new_raw, &ambient_old_raw,
                                    &object_new_raw, &object_old_raw);
    if (ret < 0) return MLX_TEMP_INVALID;
    *ambient = mlx90632_calc_temp_ambient(ambient_new_raw, ambient_old_raw, mlx_cali_PT, mlx_cali_PR, mlx_cali_PG, mlx_cali_PO, mlx_cali_Gb);
    double pre_ambient = mlx90632_preprocess_temp_ambient(ambient_new_raw, ambient_old_raw, mlx_cali_Gb);
    double pre_object = mlx90632_preprocess_temp_object(object_new_raw, object_old_raw,ambient_new_raw, ambient_old_raw, mlx_cali_Ka);
    *object = mlx90632_calc_temp_object(pre_object, pre_ambient, mlx_cali_Ea, mlx_cali_Eb, mlx_cali_Ga, mlx_cali_Fa, mlx_cali_Fb, mlx_cali_Ha, mlx_cali_Hb);

    //ESP_LOGI(TAG, "MLX90632 Run");
    return *object;
}

double mlx_measure(){
  double object, ambient;
  mlx_measure(&object, &ambient);
  return object;
}

double mlx_measure(double* obj, double* amb, double* obj_r, int16_t* a1, int16_t* a2, int16_t* a3, int16_t* a4){
  int32_t ret = 0; /**< Variable will store return values */
  // double ambient; /**< Ambient temperature in degrees Celsius */
  // double object; /**< Object temperature in degrees Celsius */
  int16_t ambient_new_raw = 120;
  int16_t ambient_old_raw = 320;
  int16_t object_new_raw = 3210;
  int16_t object_old_raw = 1230;
  double pre_ambient, pre_object, ambient, object, obj2;

  *obj = MLX_TEMP_INVALID;
  *amb = MLX_TEMP_INVALID;
  *obj_r = MLX_TEMP_INVALID;
  *a1 = *a2 = *a3 = *a4 = 0;
  if (!mlx_ready()) return MLX_TEMP_INVALID;
  ret = mlx90632_read_temp_raw(&ambient_new_raw, &ambient_old_raw, &object_new_raw, &object_old_raw);
  if (ret < 0) return MLX_TEMP_INVALID;
  pre_ambient =mlx90632_preprocess_temp_ambient(ambient_new_raw, ambient_old_raw, mlx_cali_Gb);  /// AMB
  pre_object = mlx90632_preprocess_temp_object(object_new_raw, object_old_raw,ambient_new_raw, ambient_old_raw, mlx_cali_Ka); /// ST0
  ambient = mlx90632_calc_temp_ambient(ambient_new_raw, ambient_old_raw, mlx_cali_PT, mlx_cali_PR, mlx_cali_PG, mlx_cali_PO, mlx_cali_Gb);  // Ta
  object = mlx90632_calc_temp_object(pre_object, pre_ambient, mlx_cali_Ea, mlx_cali_Eb, mlx_cali_Ga, mlx_cali_Fa, mlx_cali_Fb, mlx_cali_Ha, mlx_cali_Hb);
  obj2 = mlx90632_calc_temp_object_reflected(pre_object, pre_ambient, ambient, mlx_cali_Ea, mlx_cali_Eb, mlx_cali_Ga, mlx_cali_Fa, mlx_cali_Fb, mlx_cali_Ha, mlx_cali_Hb);

  *obj = object;
  *amb = ambient;
  *obj_r = obj2;
  *a1 = ambient_new_raw;
  *a2 = ambient_old_raw;
  *a3 = object_new_raw;
  *a4 = object_old_raw;
  return obj2;
}


void mlx_print_paras(double e){

  // Serial.printf("%d,%d,%d,%d,%d,%d,%d,%d\n",mlx_cali_PR,mlx_cali_PG,mlx_cali_PT,mlx_cali_PO,mlx_cali_Ea,mlx_cali_Eb,mlx_cali_Fa,mlx_cali_Fb);
  // Serial.printf("%d,%d,%d,%d,%d\n",mlx_cali_Ga,mlx_cali_Ha,mlx_cali_Hb,mlx_cali_Gb,mlx_cali_Ka);

  int32_t ret = 0; /**< Variable will store return values */
  // double ambient; /**< Ambient temperature in degrees Celsius */
  // double object; /**< Object temperature in degrees Celsius */
  int16_t ambient_new_raw = 120;
  int16_t ambient_old_raw = 320;
  int16_t object_new_raw = 3210;
  int16_t object_old_raw = 1230;

  double pre_ambient, pre_object, ambient, object, obj2;
  mlx90632_set_emissivity(e);



  unsigned int timer = millis();
  while (millis() - timer < 25000){
    ret = mlx90632_read_temp_raw(&ambient_new_raw, &ambient_old_raw, &object_new_raw, &object_old_raw);
    pre_ambient = mlx90632_preprocess_temp_ambient(ambient_new_raw, ambient_old_raw, mlx_cali_Gb);  /// AMB
    pre_object = mlx90632_preprocess_temp_object(object_new_raw, object_old_raw,ambient_new_raw, ambient_old_raw, mlx_cali_Ka); /// ST0
    ambient = mlx90632_calc_temp_ambient(ambient_new_raw, ambient_old_raw, mlx_cali_PT, mlx_cali_PR, mlx_cali_PG, mlx_cali_PO, mlx_cali_Gb);  // Ta
    object = mlx90632_calc_temp_object(pre_object, pre_ambient, mlx_cali_Ea, mlx_cali_Eb, mlx_cali_Ga, mlx_cali_Fa, mlx_cali_Fb, mlx_cali_Ha, mlx_cali_Hb);
    obj2 = mlx90632_calc_temp_object_reflected(pre_object, pre_ambient, ambient, mlx_cali_Ea, mlx_cali_Eb, mlx_cali_Ga, mlx_cali_Fa, mlx_cali_Fb, mlx_cali_Ha, mlx_cali_Hb);


    Serial.printf("%d,%d,%d,%d,%f,%f,%f,%f\n",ambient_new_raw,ambient_old_raw,object_new_raw,object_old_raw, pre_ambient, ambient, object,obj2);
  }

  //ret = mlx90632_read_temp_raw(&ambient_new_raw, &ambient_old_raw, &object_new_raw, &object_old_raw);

  
  //Serial.printf("%d,%d,%d,%d,%d\n",ambient_new_raw,ambient_old_raw,object_new_raw,object_old_raw);

  // double object, ambient;
  // mlx_measure(&object, &ambient);

  // Serial.printf("%f, %f\n", object, ambient);

}


// mlxee: the EEPROM block 0x2400-0x24FF as checked single-word reads (---- = failed
// transaction), <reps> passes (0 = skip the dump); then, for each 32-bit constant, the
// word-pair value next to the pre-fix 4-byte burst read (write without stop + 4-byte read)
// so the two read paths can be compared on the same sensor; then what mlx_init() kept,
// the attempts it needed and the bus errors since boot.
void mlx_dump_eeprom(uint8_t reps){
  for (uint8_t r = 0; r < reps; r++){
    for (uint16_t row = 0x2400; row < 0x2500; row += 16){
      Serial.printf("EE%u %04X:", r, row);
      for (uint16_t a = row; a < row + 16; a++){
        uint16_t v = 0;
        if (mlx90632_i2c_read(a, &v) == 0) Serial.printf(" %04X", v);
        else Serial.print(" ----");
      }
      Serial.print("\n");
    }
  }
  static const uint16_t k32[] = {MLX90632_EE_P_R, MLX90632_EE_P_G, MLX90632_EE_P_T, MLX90632_EE_P_O,
                                 MLX90632_EE_Ea, MLX90632_EE_Eb, MLX90632_EE_Fa, MLX90632_EE_Fb, MLX90632_EE_Ga};
  for (uint8_t i = 0; i < sizeof(k32) / sizeof(k32[0]); i++){
    int32_t words = 0;
    const bool ok = mlx_ee_read32(k32[i], &words);
    uint8_t addr[2] = {(uint8_t)(k32[i] >> 8), (uint8_t)(k32[i] & 0x00FF)};
    uint8_t burst[4] = {0, 0, 0, 0};
    const bool burst_ok = mlx != NULL && mlx->write(addr, 2, false) && mlx->read(burst, 4);
    const uint32_t burst_value = ((uint32_t)((burst[2] << 8) | burst[3]) << 16) | (uint16_t)((burst[0] << 8) | burst[1]);
    Serial.printf("C32 %04X words:%08lX%s burst:%08lX%s\n", k32[i], (unsigned long)(uint32_t)words, ok ? "" : "(err)",
                  (unsigned long)burst_value, burst_ok ? "" : "(err)");
  }
  Serial.printf("STATE attempts:%u bus_errors:%lu\n", mlx_calibration_attempts, (unsigned long)mlx_bus_errors);
  Serial.printf("KEPT ok:%u PR:%08lX PG:%08lX PT:%08lX PO:%08lX Ea:%08lX Eb:%08lX Fa:%08lX Fb:%08lX Ga:%08lX Gb:%04X Ka:%04X Ha:%04X Hb:%04X\n",
                mlx_calibration_ok,
                (unsigned long)mlx_cali_PR, (unsigned long)mlx_cali_PG, (unsigned long)mlx_cali_PT, (unsigned long)mlx_cali_PO,
                (unsigned long)mlx_cali_Ea, (unsigned long)mlx_cali_Eb, (unsigned long)mlx_cali_Fa, (unsigned long)mlx_cali_Fb,
                (unsigned long)mlx_cali_Ga, (uint16_t)mlx_cali_Gb, (uint16_t)mlx_cali_Ka, (uint16_t)mlx_cali_Ha, (uint16_t)mlx_cali_Hb);
}

// Feeds ambit_calibration_local.mlx_coef (boot banner, cmd 33, cal_version). Without a
// verified set it reports zeros, not the Melexis placeholders, so "no calibration" is
// visible to every host.
void mlx_read_coe(int32_t* arr){
  if (!mlx_calibration_ok){
    for (uint8_t i = 0; i < 14; i++) arr[i] = 0;
    return;
  }
  arr[0] = mlx_cali_PR;
  arr[1] = mlx_cali_PG;
  arr[2] = mlx_cali_PT;
  arr[3] = mlx_cali_PO;
  arr[4] = mlx_cali_Ea;
  arr[5] = mlx_cali_Eb;
  arr[6] = mlx_cali_Fa;
  arr[7] = mlx_cali_Fb;
  arr[8] = mlx_cali_Ga;
  arr[9] = mlx_cali_Ha;
  arr[10] = mlx_cali_Hb;
  arr[11] = mlx_cali_Gb;
  arr[12] = mlx_cali_Ka;
  arr[13] = 0;
  for (uint8_t i = 0; i < 13; i++){
    arr[13] += arr[i];
  }
  return;  
}