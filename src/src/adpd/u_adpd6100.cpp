#include "u_adpd6100.h"

static const char* TAG = "ADPD";


ADPD6::ADPD6(void){
    ADPD6::adpd1_spi_handle= NULL;   // ESP32 Spi device handle

    ADPD6::adpd_spi_dev = { // spi configuration
        .command_bits = 0, 
        .address_bits = 16, 
        .dummy_bits = 0, 
        .mode = 3, 
        .clock_speed_hz = SPI_MASTER_FREQ_10M,
        .spics_io_num = ADPD1_CS, 
        .queue_size=1
    };
    ADPD6::driver.set_transport({&adpd1_spi_handle,
                                jii_adpd6000_spi_transact});

    ADPD6::load_default(&(ADPD6::sys_config));
    ADPD6::load_default(&(ADPD6::led_config));
    ADPD6::load_default(&(ADPD6::SNR_config));
    ADPD6::load_default(&(ADPD6::DI_config));
    ADPD6::load_default(&(ADPD6::AI_config));
    ADPD6::load_default(&(ADPD6::signal_config));
};

ADPD6::~ADPD6(void){

};


bool ADPD6::begin(void){
    esp_err_t err = 1;
    if (_spi_device_attached){
      spi_bus_remove_device(ADPD6::adpd1_spi_handle);
      ADPD6::_spi_device_attached = false;
      ADPD6::chip_check = false;
      ESP_LOGI(TAG, "ADPD reset");
    }

    ESP_LOGV(TAG, "ADPD Initate:");

    err = spi_bus_add_device(SPI2_HOST, &(ADPD6::adpd_spi_dev), &(ADPD6::adpd1_spi_handle));
    if (err != ESP_OK){
        ESP_LOGE(TAG, "SPI Bus error: %s", esp_err_to_name(err));
        return false;
    }
    ESP_LOGI(TAG, "ADPD attached as SPI device");

    ADPD6::_spi_device_attached = true;
    uint8_t ret1 = 0;
    uint8_t ret2 = 0;
    err = static_cast<esp_err_t>(ADPD6::driver.identify(&ret1, &ret2));
    if (err != ESP_OK) {
      ADPD6::chip_check = false;
      ESP_LOGE(TAG, "ADPD identity read failed: %s", esp_err_to_name(err));
      return false;
    }

    if (ret1 == 0xC4) {
      ADPD6::chip_check = true;
      Serial.printf("ADPD Found, chip version: %d\n", ret2);
      err = static_cast<esp_err_t>(ADPD6::driver.software_reset());
      if (err != ESP_OK) {
        ADPD6::chip_check = false;
        return false;
      }
      delay(1);
      ADPD6::load_default(&(ADPD6::sys_config));
      if (ADPD6::global_setup(&(ADPD6::sys_config)) != jii::adpd6000::kOk) {
        ADPD6::chip_check = false;
        return false;
      }
      return true;
    }else{
      ADPD6::chip_check = false;
      ESP_LOGE(TAG, "ADPD not found");
      return false;
    }
    return false;
    
}

int32_t ADPD6::clear_fifo(void){
  return ADPD6::driver.clear_fifo();
}

int32_t ADPD6::run_freq(uint32_t freq){
  return ADPD6::driver.set_slot_frequency(960000, freq);
}

int32_t ADPD6::RUN(){
  return ADPD6::driver.set_running(true);
}

int32_t ADPD6::STOP(){
  return ADPD6::driver.set_running(false);
}

int32_t ADPD6::fifo_count(uint16_t *count){
  if (count == nullptr) return jii::adpd6000::kInvalidArgument;
  *count = 0;
  const int32_t result = ADPD6::driver.fifo_byte_count(count);
  if (result != jii::adpd6000::kOk) return result;
  if (*count > jii::adpd6000::kFifoCapacityBytes){
    *count = 0;
    return jii::adpd6000::kOutOfRange;
  }
  return jii::adpd6000::kOk;
}

uint16_t ADPD6::fifo_count(){
  uint16_t count = 0;
  return fifo_count(&count) == jii::adpd6000::kOk ? count : 0;
}
int32_t ADPD6::readfifo(uint16_t num_samples, uint8_t width, uint32_t* data){
  return ADPD6::driver.read_fifo_samples(num_samples, width, data);
}

int32_t ADPD6::write_reg(uint32_t reg_addr, uint16_t *reg_data){
    if (reg_data == nullptr || reg_addr > jii::adpd6000::kMaxRegisterAddress) {
      return jii::adpd6000::kInvalidArgument;
    }
    return ADPD6::driver.write_register(static_cast<uint16_t>(reg_addr),
                                        *reg_data);
}

int32_t ADPD6::read_reg(uint32_t reg_addr){
  uint16_t reg_data = 0;
  if (reg_addr > jii::adpd6000::kMaxRegisterAddress) {
    return jii::adpd6000::kInvalidArgument;
  }
  const int32_t result = ADPD6::driver.read_register(
      static_cast<uint16_t>(reg_addr), &reg_data);
  if (result != jii::adpd6000::kOk) return result;
  return reg_data;
}
int32_t ADPD6::num_ts(uint8_t mode){
  return ADPD6::driver.set_optical_slot_count(mode);
}


void ADPD6::load_default(struct ts_led *ts){
    ts->driver1_current = 0;
    ts->driver2_current = 0;
    ts->led1_channel = LED_A;
    ts->led2_channel = LED_A;
    ts->led1_mode = 0;
    ts->led2_mode = 0;
}

int32_t ADPD6::ts_setup(jii::adpd6000::Slot timeslot_no, struct ts_led *init)
{
	int32_t ret;
	uint16_t data = 0;

	/* Set LED power and A/B channels*/
    data =  init->led2_channel << 15|
            init->driver2_current << 8|
            init->led1_channel << 7|
            init->driver1_current << 0;

	ret = ADPD6::write_reg(jii::adpd6000::reg::for_slot(jii::adpd6000::reg::kLedPower12A, timeslot_no), &data);
	if(ret != jii::adpd6000::kOk)
		return ret;

    /* Set LED power and A/B channels*/
    data =  init->led2_mode << 1|
            init->led1_mode << 0;

	ret = ADPD6::write_reg(jii::adpd6000::reg::for_slot(jii::adpd6000::reg::kLedModeA, timeslot_no), &data);
	if(ret != jii::adpd6000::kOk)
		return ret;
	return jii::adpd6000::kOk;
}


void ADPD6::load_default(struct ts_SNR *ts){
    ts->ch1_dac = 0;
    ts->ch1_led_offset = 0;
    ts->ch2_dac = 0;    
    ts->ch2_led_offset = 0;
    ts->TIA_ceiling_det = 0;
    ts->Ch2_R_int = 0;
    ts->Ch1_R_int = 0;
    ts->Vref_pulse = 0;
    ts->Vref_VPD = 2;
    ts->Vref_VPD_pulse = 3;
    ts->TIA_gain_CH2 = 1;
    ts->TIA_gain_CH1 = 1;
    ts->C_int_CH2 = 0;
    ts->C_int_CH1 = 0;
}


int32_t ADPD6::ts_setup(jii::adpd6000::Slot timeslot_no, struct ts_SNR *init)
{
	int32_t ret;
	uint16_t data = 0;

	/* Set TIA AC Dac for channel 1*/
    data =  init->ch1_dac << 7|
            init->ch1_led_offset << 0;

	ret = ADPD6::write_reg(jii::adpd6000::reg::for_slot(jii::adpd6000::reg::kAfeDac1A, timeslot_no), &data);
	if(ret != jii::adpd6000::kOk)
		return ret;

    /*  Set TIA AC Dac for channel 1*/
    data =  init->ch2_dac << 7|
            init->ch2_led_offset << 0;

	ret = ADPD6::write_reg(jii::adpd6000::reg::for_slot(jii::adpd6000::reg::kAfeDac2A, timeslot_no), &data);
	if(ret != jii::adpd6000::kOk)
		return ret;


    data =  init->TIA_ceiling_det << 15|
            init->Ch2_R_int << 13|
            init->Ch1_R_int << 11|
            init->Vref_pulse << 10|
            init->Vref_VPD << 8|
            init->Vref_pulse << 6|
            init->TIA_gain_CH2 << 3|
            init->TIA_gain_CH1 << 0;

	ret = ADPD6::write_reg(jii::adpd6000::reg::for_slot(jii::adpd6000::reg::kAfeTrim1A, timeslot_no), &data);
	if(ret != jii::adpd6000::kOk)
		return ret;

    data = 0;
    data =  init->C_int_CH2 << 13|
            init->C_int_CH1 << 12;

	ret = ADPD6::write_reg(jii::adpd6000::reg::for_slot(jii::adpd6000::reg::kAfeTrim2A, timeslot_no), &data);
	if(ret != jii::adpd6000::kOk)
		return ret;
	return jii::adpd6000::kOk;
}

void ADPD6::load_default(struct ts_signal *ts){
    ts->pre_condition_width = 4;
    ts->ac_type = 0;
    ts->gpio_out = 0;
    ts->INT2BUT = 0;
    ts->AFE_PATh = 0x28; // 0x28
    ts->IN34 = 0;
    ts->IN12 = 1;
    ts->pre_condition_type = 0;
    ts->VC2_pulse_type = 0;
    ts->VC2_pulse_to = 0;
    ts->VC2_active = 2;
    ts->VC1_pulse_type = 0;
    ts->VC1_pulse_to = 0;
    ts->VC1_active = 2;         // set vc1 to vref + 215mV
}


int32_t ADPD6::ts_setup(jii::adpd6000::Slot timeslot_no, struct ts_signal *init)
{
	int32_t ret;
	uint16_t data = 0;

	/* Set TIA AC Dac for channel 1*/
    data =  init->pre_condition_width << 12|
            init->ac_type << 10|
            init->gpio_out << 9|
            init->INT2BUT << 8|
            init->AFE_PATh << 0;

	ret = ADPD6::write_reg(jii::adpd6000::reg::for_slot(jii::adpd6000::reg::kTimeslotPathA, timeslot_no), &data);
	if(ret != jii::adpd6000::kOk)
		return ret;

    /*  Set TIA AC Dac for channel 1*/
    data =  init->IN34 << 4|
            init->IN12 << 0;

	ret = ADPD6::write_reg(jii::adpd6000::reg::for_slot(jii::adpd6000::reg::kInputsA, timeslot_no), &data);
	if(ret != jii::adpd6000::kOk)
		return ret;


    data =  init->pre_condition_type << 12|
            init->VC2_pulse_type << 10|
            init->VC2_pulse_to << 8|
            init->VC2_active << 6|
            init->VC1_pulse_type << 4|
            init->VC1_pulse_to << 2|
            init->VC1_active << 0;

	ret = ADPD6::write_reg(jii::adpd6000::reg::for_slot(jii::adpd6000::reg::kCathodeA, timeslot_no), &data);
	if(ret != jii::adpd6000::kOk)
		return ret;
	return jii::adpd6000::kOk;
}

void ADPD6::load_default(struct ts_DI_timing *ts){
  ts->num_integration = 1;
  ts->num_repeats = 1;
  ts->Modulation_type = 0;
  ts->period_min = 0;
  ts->LED_pulse_width = 2;
  ts->LED_pulse_offset = 0x10;
  ts->LED_pulse2_offset = 0x13;
  ts->LIT_OFFSET = 0x26;
  ts->DARK_OFFSET1 = 0x06;
  ts->DARK_OFFSET2 = 0x40;
  ts->subsample_en = 0;
  ts->ch2_en = 0;
  ts->sample_type = 3;   // two-region digital integration
  ts->ts_offset = 0;
  ts->dark_shift = 0;
  ts->dark_size = 0;
  ts->signal_shift = 0;
  ts->signal_size = 3;
  ts->lit_shift = 0;
  ts->lit_size = 0;
}


int32_t ADPD6::ts_setup(jii::adpd6000::Slot timeslot_no, struct ts_DI_timing *init){
    int32_t ret;
	uint16_t data = 0;


  data =  init->num_integration << 8|
            init->num_repeats << 0;
	ret = ADPD6::write_reg(jii::adpd6000::reg::for_slot(jii::adpd6000::reg::kCountsA, timeslot_no), &data);
	if(ret != jii::adpd6000::kOk)
  return ret;

    data =  init->Modulation_type << 12|
            init->period_min << 0;
	ret = ADPD6::write_reg(jii::adpd6000::reg::for_slot(jii::adpd6000::reg::kPeriodA, timeslot_no), &data);
	if(ret != jii::adpd6000::kOk)
  return ret;

    data = 0;
    data = init->LED_pulse_width << 8|
            init->LED_pulse_offset << 0;
	ret = ADPD6::write_reg(jii::adpd6000::reg::for_slot(jii::adpd6000::reg::kLedPulse1A, timeslot_no), &data);
	if(ret != jii::adpd6000::kOk)
		return ret;
    data = 0;
    data = init->LED_pulse2_offset << 0;
	ret = ADPD6::write_reg(jii::adpd6000::reg::for_slot(jii::adpd6000::reg::kLedPulse2A, timeslot_no), &data);
	if(ret != jii::adpd6000::kOk)
		return ret;

    data = 0;
    data =  init->LIT_OFFSET << 0;
    ret = ADPD6::write_reg(jii::adpd6000::reg::for_slot(jii::adpd6000::reg::kDigitalIntegrationLitA, timeslot_no), &data);
	if(ret != jii::adpd6000::kOk)
		return ret;

    data = 0;
    data =  init->DARK_OFFSET2 << 7|
            init->DARK_OFFSET1 << 0;
    ret = ADPD6::write_reg(jii::adpd6000::reg::for_slot(jii::adpd6000::reg::kDigitalIntegrationDarkA, timeslot_no), &data);
	if(ret != jii::adpd6000::kOk)
		return ret;

    data = 0;
    data =  init->subsample_en << 15|
            init->ch2_en << 14|
            init->sample_type << 11|
            init->ts_offset << 0;
    ret = ADPD6::write_reg(jii::adpd6000::reg::for_slot(jii::adpd6000::reg::kTimeslotControlA, timeslot_no), &data);
	if(ret != jii::adpd6000::kOk)
		return ret;

    data = 0;
    data =  init->dark_shift << 11|
            init->dark_size << 8|
            init->signal_shift << 3|
            init->signal_size << 0;
    ret = ADPD6::write_reg(jii::adpd6000::reg::for_slot(jii::adpd6000::reg::kData1A, timeslot_no), &data);
	if(ret != jii::adpd6000::kOk)
		return ret;      

    data = 0;
    data =  init->lit_shift << 3|
            init->lit_size << 0;
    ret = ADPD6::write_reg(jii::adpd6000::reg::for_slot(jii::adpd6000::reg::kData2A, timeslot_no), &data);
	if(ret != jii::adpd6000::kOk)
		return ret;
    return jii::adpd6000::kOk;
}

void ADPD6::load_default(struct ts_AI_timing *ts){
  ts->num_integration = 1;
  ts->num_repeats = 1;
  ts->Modulation_type = 0;
  ts->period_min = 0;
  ts->LED_pulse_width = 2;
  ts->LED_pulse_offset = 0x10;
  ts->LED_pulse2_offset = 0x13;
  ts->SINGLE_INTEG = 0;
  ts->CH2_DIS_AMP = 0;
  ts->CH1_DIS_AMP = 0;
  ts->ADC_count = 0;
  ts->integration_width = 3;
  ts->offset_us = 0xD;
  ts->offset_ns = 0;
  ts->pattern_LED_disable = 0;
  ts->pattern_MOD_disable = 0;
  ts->pattern_subtract = 0;
  ts->pattern_reverse_int = 0;
  ts->subsample_en = 0;
  ts->ch2_en = 0;
  ts->sample_type = 0;
  ts->ts_offset = 0;

} 

int32_t ADPD6::ts_setup(jii::adpd6000::Slot timeslot_no, struct ts_AI_timing *init){
    int32_t ret;
	uint16_t data = 0;

    data =  init->Modulation_type << 12|
            init->period_min << 0;
	ret = ADPD6::write_reg(jii::adpd6000::reg::for_slot(jii::adpd6000::reg::kPeriodA, timeslot_no), &data);
	if(ret != jii::adpd6000::kOk)
		return ret;

    data = 0;
    data = init->LED_pulse_width << 8|
            init->LED_pulse_offset << 0;
	ret = ADPD6::write_reg(jii::adpd6000::reg::for_slot(jii::adpd6000::reg::kLedPulse1A, timeslot_no), &data);
	if(ret != jii::adpd6000::kOk)
		return ret;
    data = 0;
    data = init->LED_pulse2_offset << 0;
	ret = ADPD6::write_reg(jii::adpd6000::reg::for_slot(jii::adpd6000::reg::kLedPulse2A, timeslot_no), &data);
	if(ret != jii::adpd6000::kOk)
		return ret;

    //0x012E INTEG_WIDTH_A
    data = 0;
    data =  init->SINGLE_INTEG << 15|
            init->CH2_DIS_AMP << 11|
            init->CH1_DIS_AMP << 7|
            init->ADC_count << 5|
            init->integration_width << 0;
    ret = ADPD6::write_reg(jii::adpd6000::reg::for_slot(jii::adpd6000::reg::kIntegratorWidthA, timeslot_no), &data);
	if(ret != jii::adpd6000::kOk)
		return ret;    

    //0x012F INTEG_OFFSET_A
    data = 0;
    data =  init->offset_us << 5|
            init->offset_ns << 0;
    ret = ADPD6::write_reg(jii::adpd6000::reg::for_slot(jii::adpd6000::reg::kIntegratorOffsetA, timeslot_no), &data);
	if(ret != jii::adpd6000::kOk)
		return ret; 

    //0x130 MOD_PULSE_A
    data = 0;
    data =  init->modulation_width << 8|
            init->modulation_offset << 0;
    ret = ADPD6::write_reg(jii::adpd6000::reg::for_slot(jii::adpd6000::reg::kModulationPulseA, timeslot_no), &data);
	if(ret != jii::adpd6000::kOk)
		return ret; 

    //0x0131 PATTERN1_A    
    data = 0;
    data =  init->pattern_LED_disable << 12|
            init->pattern_MOD_disable << 8|
            init->pattern_subtract << 4|
            init->pattern_reverse_int << 0;
    ret = ADPD6::write_reg(jii::adpd6000::reg::for_slot(jii::adpd6000::reg::kPattern1A, timeslot_no), &data);
	if(ret != jii::adpd6000::kOk)
		return ret;        


    //0x0120 TS_CTRL_A
    data = 0;
    data =  init->subsample_en << 15|
            init->ch2_en << 14|
            init->sample_type << 11|
            init->ts_offset << 0;
    ret = ADPD6::write_reg(jii::adpd6000::reg::for_slot(jii::adpd6000::reg::kTimeslotControlA, timeslot_no), &data);
	if(ret != jii::adpd6000::kOk)
		return ret;
    return jii::adpd6000::kOk;
}

void ADPD6::load_default(struct system_config *ts){
  ts->IN34_sleep = 0;
  ts->IN12_sleep = 0;
  ts->VC2_sleep = 0;
  ts->VC1_sleep = 0;
  ts->PAIR34 = 0;
  ts->PAIR12 = 0;
  ts->GPIO_slew = 0;
  ts->GPIO_drv = 0;
  ts->GPIO2_cfg = 0;
  ts->GPIO1_cfg = 0;
  ts->GPIO0_cfg = 0;
  ts->GPIO1_output = 0;
  ts->GPIO0_output = 0;
  ts->GPIO2_output = 0;
  ts->IOVDD = 0;
  ts->SPI_drv = 0;
  ts->SPI_slew = 0;
  ts->alt_clock = 0;
  ts->alt_clock_gpio = 0;
  ts->ext_clock_freq = 0;
  ts->internal_clock_en = 1;
}


int32_t ADPD6::global_setup(struct system_config *init){
    int32_t ret;
	uint16_t data = 0;

    data = 0x2004;
    ret = ADPD6::write_reg(jii::adpd6000::reg::kAdcControl, &data);
    if(ret != jii::adpd6000::kOk)
		return ret;

    data = 0x400b;
    ret = ADPD6::write_reg(jii::adpd6000::reg::kGlobalBiasControl, &data);
    if(ret != jii::adpd6000::kOk)
		return ret;

    data = 0;
    data =  init->IN34_sleep << 4|
            init->IN12_sleep << 0;
    ret = ADPD6::write_reg(jii::adpd6000::reg::kInputSleep, &data);
	if(ret != jii::adpd6000::kOk)
		return ret;


    data = 0;
    data =  init->VC2_sleep << 6|
            init->VC1_sleep << 4|
            init->PAIR34 << 1|
            init->PAIR12 << 0;
    ret = ADPD6::write_reg(jii::adpd6000::reg::kInputConfig, &data);
	if(ret != jii::adpd6000::kOk)
		return ret;
    
    data = 0;
    data =  init->GPIO_slew << 14|
            init->GPIO_drv << 12|
            init->GPIO2_cfg << 6|
            init->GPIO1_cfg << 3|
            init->GPIO0_cfg << 0;
    ret = ADPD6::write_reg(jii::adpd6000::reg::kGpioConfig, &data);
	if(ret != jii::adpd6000::kOk)
		return ret;


    data = 0;
    data =  init->GPIO1_output << 8|
            init->GPIO0_output << 0;
    ret = ADPD6::write_reg(jii::adpd6000::reg::kGpio01, &data);
	if(ret != jii::adpd6000::kOk)
		return ret;

    data = 0;
    data =  init->GPIO2_output << 0;
    ret = ADPD6::write_reg(jii::adpd6000::reg::kGpio23, &data);
	if(ret != jii::adpd6000::kOk)
		return ret; 
       
    data = 0;
    data =  init->IOVDD << 6|
            1 << 4|
            init->SPI_slew << 2|
            init->SPI_drv << 0;
    ret = ADPD6::write_reg(jii::adpd6000::reg::kIoAdjust, &data);
	if(ret != jii::adpd6000::kOk)
		return ret; 

    data = 0;
    data =  init->alt_clock << 8|
            init->alt_clock_gpio << 6|
            init->ext_clock_freq << 2|
            init->internal_clock_en << 1;
    ret = ADPD6::write_reg(jii::adpd6000::reg::kSystemControl, &data);
	if(ret != jii::adpd6000::kOk)
		return ret;
    return jii::adpd6000::kOk;
}

void ADPD6::load_default(struct GPIO0_config *ts){
  ts->GPIO0_cfg = 0;
  ts->GPIO0_output = 0;
  ts->EXT_SYNC_EN = 0;
  ts->SYNC_GPIO = 0;
}

int32_t ADPD6::gpio_setup(struct GPIO0_config *init){
  int32_t ret;
	uint16_t data = 0;
  data =  init->EXT_SYNC_EN << 2|
          init->SYNC_GPIO << 0;
	ret = ADPD6::write_reg(jii::adpd6000::reg::kGpioExternal, &data);
	if(ret != jii::adpd6000::kOk) return ret;

  data = 0;
  data = init->GPIO0_output << 0;
	ret = ADPD6::write_reg(jii::adpd6000::reg::kGpio01, &data);
	if(ret != jii::adpd6000::kOk) return ret;

  data = 0;
  data = init->GPIO0_cfg;
	ret = ADPD6::write_reg(jii::adpd6000::reg::kGpioConfig, &data);
	if(ret != jii::adpd6000::kOk) return ret;

  return 0;
}
int32_t ADPD6::preset_config_1(uint8_t ts, uint8_t num_integ){
  if (!(ADPD6::chip_check)){
    ESP_LOGE(TAG, "ADPD Not init in preset1");
    return -2;
  }

  ESP_LOGV(TAG, "Preset_config 1 set for timeslot:%d. Two ambient channels. Total 2 x 3 bytes", ts);
  // Channel 1: PD1 (sun-facing), channel 2: PD2 (leaf-facing IR)

  ADPD6::STOP();
  ADPD6::DI_config.period_min = 58;
  ADPD6::DI_config.LIT_OFFSET = 64;
  ADPD6::DI_config.DARK_OFFSET1 = 48;
  ADPD6::DI_config.DARK_OFFSET2 = 90;
  ADPD6::DI_config.LED_pulse_offset = 60;
  ADPD6::DI_config.LED_pulse_width = 19;
  ADPD6::DI_config.sample_type = 3;
  ADPD6::DI_config.signal_size = 0;
  ADPD6::DI_config.num_integration = num_integ;
  ADPD6::DI_config.num_repeats = 1;
  ADPD6::DI_config.dark_size = 3;
  ADPD6::DI_config.lit_size = 0;

  ADPD6::signal_config.pre_condition_type = 5;
  ADPD6::SNR_config.Ch1_R_int = 1;
  ADPD6::SNR_config.C_int_CH1 = 1;
  ADPD6::SNR_config.Ch2_R_int = 1;
  ADPD6::SNR_config.C_int_CH2 = 1;
  ADPD6::signal_config.INT2BUT = 1;
  ADPD6::signal_config.ac_type = 0;

  ADPD6::DI_config.ch2_en = true;
  ADPD6::signal_config.IN12 = 0B0101;
  ADPD6::signal_config.IN34 = 0B0000;
  
  ADPD6::num_ts(ts + 1);

  ADPD6::ts_setup(static_cast<jii::adpd6000::Slot>(ts), &(ADPD6::DI_config));
  ADPD6::ts_setup(static_cast<jii::adpd6000::Slot>(ts), &(ADPD6::signal_config));
  ADPD6::ts_setup(static_cast<jii::adpd6000::Slot>(ts), &(ADPD6::SNR_config));
  ADPD6::ts_setup(static_cast<jii::adpd6000::Slot>(ts), &(ADPD6::led_config));

  return 0;

}


// fluo and ref channel setup
int32_t ADPD6::preset_config_2(uint8_t ts, uint8_t num_integ){
  if (!(ADPD6::chip_check)){
    ESP_LOGE(TAG, "ADPD Not init in preset 2");
    return -2;
  }

  ESP_LOGV(TAG, "Preset_config 2 set for timeslot:%d. Total 2 x 2 x 3 bytes", ts);
  // Channel 1: PD2 (leaf-facing IR), channel 2: PD4 (leaf-facing Vis)

  ADPD6::STOP();
  ADPD6::DI_config.period_min = 58;
  ADPD6::DI_config.LIT_OFFSET = 72;//64;
  ADPD6::DI_config.DARK_OFFSET1 = 48;
  ADPD6::DI_config.DARK_OFFSET2 = 90;
  ADPD6::DI_config.LED_pulse_offset = 60;
  ADPD6::DI_config.LED_pulse_width = 15;//19;
  ADPD6::DI_config.sample_type = 3;
  ADPD6::DI_config.signal_size = 0;
  ADPD6::DI_config.num_integration = num_integ;
  ADPD6::DI_config.num_repeats = 1;
  ADPD6::DI_config.dark_size = 3;
  ADPD6::DI_config.lit_size = 3;

  ADPD6::signal_config.pre_condition_type = 5;
  ADPD6::SNR_config.Ch1_R_int = 1;
  ADPD6::SNR_config.C_int_CH1 = 1;
  ADPD6::SNR_config.Ch2_R_int = 2; // 1
  ADPD6::SNR_config.C_int_CH2 = 1;
  ADPD6::signal_config.INT2BUT = 1;
  ADPD6::signal_config.ac_type = 2;

  ADPD6::DI_config.ch2_en = true;
  ADPD6::signal_config.IN12 = 0B0011;
  ADPD6::signal_config.IN34 = 0B0100;


  
  ADPD6::num_ts(ts + 1);

  ADPD6::ts_setup(static_cast<jii::adpd6000::Slot>(ts), &(ADPD6::DI_config));
  ADPD6::ts_setup(static_cast<jii::adpd6000::Slot>(ts), &(ADPD6::signal_config));
  ADPD6::ts_setup(static_cast<jii::adpd6000::Slot>(ts), &(ADPD6::SNR_config));
  ADPD6::ts_setup(static_cast<jii::adpd6000::Slot>(ts), &(ADPD6::led_config));

  return 0;
}

int32_t ADPD6::preset_config_2x(uint8_t ts, uint8_t num_integ, uint8_t lit_offset, uint8_t dark1_offset, uint8_t dark2_offset, uint8_t pulse_offset, uint8_t pulse_duration){
  if (!(ADPD6::chip_check)){
    ESP_LOGE(TAG, "ADPD Not init in preset 2");
    return -2;
  }

  ESP_LOGV(TAG, "Preset_config 2 set for timeslot:%d. Total 2 x 2 x 3 bytes", ts);
  // Channel 1: PD2 (leaf-facing IR), channel 2: PD4 (leaf-facing Vis)

  ADPD6::STOP();
  ADPD6::DI_config.period_min = 58;
  ADPD6::DI_config.LIT_OFFSET = lit_offset;
  ADPD6::DI_config.DARK_OFFSET1 = dark1_offset;
  ADPD6::DI_config.DARK_OFFSET2 = dark2_offset;
  ADPD6::DI_config.LED_pulse_offset = pulse_offset;
  ADPD6::DI_config.LED_pulse_width = pulse_duration;
  ADPD6::DI_config.sample_type = 3;
  ADPD6::DI_config.signal_size = 0;
  ADPD6::DI_config.num_integration = num_integ;
  ADPD6::DI_config.num_repeats = 1;
  ADPD6::DI_config.dark_size = 3;
  ADPD6::DI_config.lit_size = 3;

  ADPD6::signal_config.pre_condition_type = 5;
  ADPD6::SNR_config.Ch1_R_int = 1;
  ADPD6::SNR_config.C_int_CH1 = 1;
  ADPD6::SNR_config.Ch2_R_int = 1;
  ADPD6::SNR_config.C_int_CH2 = 1;
  ADPD6::signal_config.INT2BUT = 1;
  ADPD6::signal_config.ac_type = 2;

  ADPD6::DI_config.ch2_en = true;
  ADPD6::signal_config.IN12 = 0B0011;
  ADPD6::signal_config.IN34 = 0B0100;


  
  ADPD6::num_ts(ts + 1);

  ADPD6::ts_setup(static_cast<jii::adpd6000::Slot>(ts), &(ADPD6::DI_config));
  ADPD6::ts_setup(static_cast<jii::adpd6000::Slot>(ts), &(ADPD6::signal_config));
  ADPD6::ts_setup(static_cast<jii::adpd6000::Slot>(ts), &(ADPD6::SNR_config));
  ADPD6::ts_setup(static_cast<jii::adpd6000::Slot>(ts), &(ADPD6::led_config));

  return 0;
}

// reflection channel setup
int32_t ADPD6::preset_config_3(uint8_t ts, uint8_t num_integ){
  if (!(ADPD6::chip_check)){
    ESP_LOGE(TAG, "ADPD Not init in preset 2");
    return -2;
  }

  ESP_LOGV(TAG, "Preset_config 3 set for timeslot:%d. Total 2 x 3 bytes", ts);
  // Channel 1: PD2 (leaf-facing IR), channel 2: PD4 (leaf-facing Vis)

  ADPD6::STOP();
  ADPD6::DI_config.period_min = 58;
  ADPD6::DI_config.LIT_OFFSET = 72;//64;
  ADPD6::DI_config.DARK_OFFSET1 = 48;
  ADPD6::DI_config.DARK_OFFSET2 = 90;
  ADPD6::DI_config.LED_pulse_offset = 60;
  ADPD6::DI_config.LED_pulse_width = 19;
  ADPD6::DI_config.sample_type = 3;
  ADPD6::DI_config.signal_size = 3;
  ADPD6::DI_config.num_integration = num_integ;
  ADPD6::DI_config.num_repeats = 1;
  ADPD6::DI_config.dark_size = 0;
  ADPD6::DI_config.lit_size = 0;

  ADPD6::signal_config.pre_condition_type = 5;
  ADPD6::SNR_config.Ch1_R_int = 1;
  ADPD6::SNR_config.C_int_CH1 = 1;
  ADPD6::SNR_config.Ch2_R_int = 1;
  ADPD6::SNR_config.C_int_CH2 = 1;
  ADPD6::signal_config.INT2BUT = 1;
  ADPD6::signal_config.ac_type = 2;

  ADPD6::DI_config.ch2_en = true;
  ADPD6::signal_config.IN12 = 0B0011;
  ADPD6::signal_config.IN34 = 0B0100;

  
  ADPD6::num_ts(ts + 1);

  ADPD6::ts_setup(static_cast<jii::adpd6000::Slot>(ts), &(ADPD6::DI_config));
  ADPD6::ts_setup(static_cast<jii::adpd6000::Slot>(ts), &(ADPD6::signal_config));
  ADPD6::ts_setup(static_cast<jii::adpd6000::Slot>(ts), &(ADPD6::SNR_config));
  ADPD6::ts_setup(static_cast<jii::adpd6000::Slot>(ts), &(ADPD6::led_config));

  return 0;
}



// far-red
int32_t ADPD6::preset_config_4(uint8_t ts){
  if (!(ADPD6::chip_check)){
    ESP_LOGE(TAG, "ADPD Not init in preset 3");
    return -2;
  }

  ESP_LOGV(TAG, "Preset_config 4 set for timeslot:%d. Total 0 bytes", ts);
  // Channel 1: PD2 (leaf-facing IR), channel 2: PD4 (leaf-facing Vis)

  ADPD6::STOP();
  ADPD6::DI_config.period_min = 320;
  ADPD6::DI_config.LIT_OFFSET = 100;
  ADPD6::DI_config.DARK_OFFSET1 = 48;
  ADPD6::DI_config.DARK_OFFSET2 = 300;
  ADPD6::DI_config.LED_pulse_offset = 60;
  ADPD6::DI_config.LED_pulse_width = 200;
  ADPD6::DI_config.sample_type = 3;
  ADPD6::DI_config.signal_size = 0;
  ADPD6::DI_config.num_integration = 1;
  ADPD6::DI_config.num_repeats = 1;
  ADPD6::DI_config.dark_size = 0;
  ADPD6::DI_config.lit_size = 0;

  ADPD6::signal_config.pre_condition_type = 5;
  ADPD6::SNR_config.Ch1_R_int = 1;
  ADPD6::SNR_config.C_int_CH1 = 1;
  ADPD6::SNR_config.Ch2_R_int = 1;
  ADPD6::SNR_config.C_int_CH2 = 1;
  ADPD6::signal_config.INT2BUT = 1;
  ADPD6::signal_config.ac_type = 0;

  ADPD6::DI_config.ch2_en = false;
  ADPD6::signal_config.IN12 = 0B0011;
  ADPD6::signal_config.IN34 = 0B0100;


  
  ADPD6::num_ts(ts + 1);

  ADPD6::ts_setup(static_cast<jii::adpd6000::Slot>(ts), &(ADPD6::DI_config));
  ADPD6::ts_setup(static_cast<jii::adpd6000::Slot>(ts), &(ADPD6::signal_config));
  ADPD6::ts_setup(static_cast<jii::adpd6000::Slot>(ts), &(ADPD6::SNR_config));
  ADPD6::ts_setup(static_cast<jii::adpd6000::Slot>(ts), &(ADPD6::led_config));

  return 0;
}

int32_t ADPD6::repeats_only(uint8_t ts, uint16_t num_integration, uint8_t num_repeats){
  	uint16_t data = 0;
    int32_t ret;
    data =  num_integration << 8|
              num_repeats << 0;
    ret = ADPD6::write_reg(jii::adpd6000::reg::for_slot(jii::adpd6000::reg::kCountsA, static_cast<jii::adpd6000::Slot>(ts)), &data);
    if(ret != jii::adpd6000::kOk)
    return ret;
    return jii::adpd6000::kOk;
}

// external trigger
int32_t ADPD6::preset_config_ext_fast(uint8_t ts){
  return ADPD6::preset_config_ext_fast(ts, 1);

}

int32_t ADPD6::preset_config_ext_fast(uint8_t ts, uint8_t integ){
  if (!(ADPD6::chip_check)){
    ESP_LOGE(TAG, "ADPD Not init in preset 2");
    return -2;
  }

  ESP_LOGV(TAG, "External trigger test: %d", ts);
  // Channel 1: PD2 (leaf-facing IR), channel 2: PD4 (leaf-facing Vis)

  ADPD6::STOP();
  ADPD6::DI_config.period_min = 50;
  ADPD6::DI_config.LIT_OFFSET = 70;
  ADPD6::DI_config.DARK_OFFSET1 = 48;
  ADPD6::DI_config.DARK_OFFSET2 = 85;
  ADPD6::DI_config.LED_pulse_offset = 60;
  ADPD6::DI_config.LED_pulse_width = 15;
  ADPD6::DI_config.sample_type = 3;
  ADPD6::DI_config.signal_size = 0;
  ADPD6::DI_config.num_integration = integ;
  ADPD6::DI_config.num_repeats = 1;
  ADPD6::DI_config.dark_size = 3;
  ADPD6::DI_config.lit_size = 3;

  ADPD6::signal_config.pre_condition_type = 5;
  ADPD6::SNR_config.Ch1_R_int = 1;
  ADPD6::SNR_config.C_int_CH1 = 1;
  ADPD6::SNR_config.Ch2_R_int = 2;
  ADPD6::SNR_config.C_int_CH2 = 1;
  ADPD6::signal_config.INT2BUT = 1;
  ADPD6::signal_config.ac_type = 2;

  ADPD6::DI_config.ch2_en = true;
  ADPD6::signal_config.IN12 = 0B0011;
  ADPD6::signal_config.IN34 = 0B0100;  


  
  ADPD6::num_ts(ts + 1);
  ADPD6::ts_setup(static_cast<jii::adpd6000::Slot>(ts), &(ADPD6::DI_config));
  ADPD6::ts_setup(static_cast<jii::adpd6000::Slot>(ts), &(ADPD6::signal_config));
  ADPD6::ts_setup(static_cast<jii::adpd6000::Slot>(ts), &(ADPD6::SNR_config));
  ADPD6::ts_setup(static_cast<jii::adpd6000::Slot>(ts), &(ADPD6::led_config));

  return 0;
}
