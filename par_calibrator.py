import serial
import time
import sys
def main(input_str = "", output = "", hello = False):
    with serial.Serial("COM31", 115200) as ser:
        readable = ""
        timer0 = time.perf_counter()
        ser.write(("\r\n").encode())    
        while time.perf_counter() - timer0 < 5:
            time.sleep(0.01)
            try:
                pass
            except KeyboardInterrupt:
                break
            if input_str != "":
                ser.write((input_str + "\r\n").encode())     
                print("write", input_str)
                time.sleep(0.1)
                if output == "": return 0
            
            while ser.in_waiting > 0:                
                _c = ser.read()
                if (_c > bytes([128]) or _c < bytes([32])) and not (_c == b"\r" or _c == b"\n"or _c == b"\t"):
                    print(int.from_bytes(_c))
                    if hello:
                        if int.from_bytes(_c) == 133:
                            ser.write("hello\r\n".encode())
                            print("write hello")
                else:
                    readable += _c.decode()
                    if _c == b"\n":
                        print(readable, end="")
                        if output != "":
                            if readable.startswith(output):
                                return 1
                        readable = ""
            

def cali_par(ground_truth = 1000):
    with serial.Serial("COM31", 115200) as ser:
        readable = ""           

        
        ser.write("get_par".encode())     
        print("write get_par")
        timer0 = time.perf_counter()   
        while time.perf_counter() - timer0 < 1:
            time.sleep(0.01)
            if ser.in_waiting > 0:
                break
        
        output = ""
        line_count = 0
        while ser.in_waiting > 0:
            _c = ser.read()
            if (_c > bytes([128]) or _c < bytes([32])) and not (_c == b"\r" or _c == b"\n"or _c == b"\t"):
                print(int.from_bytes(_c))
                
            else:
                readable += _c.decode()
                if _c == b"\n":
                    print(line_count, readable, end="")
                    if (line_count == 0): output = readable.strip()
                    line_count += 1
                    readable = ""
                    
        if line_count == 2:
            try:
                meas = float(output)
                print("get readings")
                
                if (meas > 0):
                    print(f'{ground_truth / meas:.6f}')
                    if (ground_truth / meas > 0.65 or ground_truth / meas < 0.1):
                        print("calibration failed")
                        return 0
                    ser.write(f"set_spec,{ground_truth / meas:.6f}".encode())
                
                
            except:
                print("get readings failed")
                return 0
            
            
    






                
if __name__ == "__main__":
    par = 0
    if len(sys.argv) == 3:
        print("PAR:", sys.argv[1])
        par = int(sys.argv[1])
        ambit_name = sys.argv[2]
    else:
        print("PAR: 0")
        
        
            
    if main("hello,", "NEW Name Here", True) == 1:
        print("hello")
        main("set_name,"+ambit_name, "")
        time.sleep(0.25)
        main("set_emit,0.9", "")
        time.sleep(0.25)
        main("set_act,0.1", "")
        time.sleep(0.25)
        if par == 0:
            cali_par(ground_truth = 0)
        else:
            cali_par(ground_truth = par)

        input("Press Enter to do baseline")
        main("baseline,1", "")
        time.sleep(2)
        main("hello,", "NEW Name Here")
        
    