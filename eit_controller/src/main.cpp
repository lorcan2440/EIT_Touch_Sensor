#include <Arduino.h>
#include <EIT.h>
#include <vector>


void setup() 
{
    Serial.begin(115200);

    while(!Serial);

    pinMode(HSPI_MOSI_PIN, OUTPUT);
    pinMode(HSPI_SCK_PIN, OUTPUT);
    pinMode(VSPI_MOSI_PIN, OUTPUT);
    pinMode(VSPI_SCK_PIN, OUTPUT);
    
    pinMode(CHIP_SEL_DRIVE, OUTPUT);
    pinMode(CHIP_SEL_MEAS, OUTPUT);
    pinMode(CHIP_SEL_MUX_SRC, OUTPUT);
    pinMode(CHIP_SEL_MUX_SINK, OUTPUT);
    pinMode(CHIP_SEL_MUX_VP, OUTPUT);
    pinMode(CHIP_SEL_MUX_VN, OUTPUT);
    pinMode(CHIP_SEL_AD5930, OUTPUT);
    
    pinMode(AD5930_INT_PIN, OUTPUT);
    pinMode(AD5930_CTRL_PIN, OUTPUT);
    pinMode(AD5930_STANDBY_PIN, OUTPUT);
    pinMode(AD5930_MSBOUT_PIN, INPUT);

    // ADC input
    pinMode(14, INPUT);
    pinMode(15, INPUT);
    pinMode(16, INPUT);
    pinMode(17, INPUT);
    pinMode(18, INPUT);
    pinMode(19, INPUT);
    pinMode(20, INPUT);
    pinMode(21, INPUT);
    pinMode(22, INPUT);
    pinMode(23, INPUT);

    digitalWrite(CHIP_SEL_DRIVE, HIGH);
    digitalWrite(CHIP_SEL_MEAS, HIGH);
    digitalWrite(CHIP_SEL_MUX_SRC, HIGH);
    digitalWrite(CHIP_SEL_MUX_SINK, HIGH);
    digitalWrite(CHIP_SEL_MUX_VP, HIGH);
    digitalWrite(CHIP_SEL_MUX_VN, HIGH);
    digitalWrite(CHIP_SEL_AD5930, HIGH);
    digitalWrite(AD5930_INT_PIN, LOW);
    digitalWrite(AD5930_CTRL_PIN, LOW);
    digitalWrite(AD5930_STANDBY_PIN, LOW);
    
    digitalWrite(ADS_PWR, LOW); //double-check
    digitalWrite(ADS_OE, LOW);

//    SPI.begin();

    /* B24 = 0 (start freq high and low regs can be written independently)
    * DAC ENABLE = 1 (DAC enabled)
    * SINE/TRI = 1 (sine output)
    * MSBOUTEN = 1 (MSBOUT enabled)
    * CW/BURST = 1 (no mid-scale output after burst)
    * INT/EXT BURST = 1 (burst controlled by CTRL pin)
    * INT/EXT INCR = 1 (frequency increment controlled by CTRL pin)
    * MODE = 1 (frequency saw sweep)
    * SYNCSEL = 0 (SYNCOUT outputs pulse at each freq increment)
    * SYNCOUTEN = 0 (SYNCOUT disabled)
    */
    AD5930_Write(CTRL_REG, 0b011111110011);
    AD5930_Set_Start_Freq(TEST_FREQ);

    AD5270_Lock(CHIP_SEL_DRIVE, 0);
    AD5270_Lock(CHIP_SEL_MEAS, 0);

    /* Start the frequency sweep */
    digitalWrite(AD5930_CTRL_PIN, HIGH);
    delay(100);

    calibrate_samples();
    // calibrate_gain(AD, AD);
//    AD5270_Set(CHIP_SEL_MEAS, 100);  // fingertip
//    AD5270_Set(CHIP_SEL_DRIVE, 500);  // fingertip
     AD5270_Set(CHIP_SEL_MEAS, 100);  // lorcan
     AD5270_Set(CHIP_SEL_DRIVE, 700);  // lorcan
//    AD5270_Set(CHIP_SEL_MEAS, 100);  // hand
//    AD5270_Set(CHIP_SEL_DRIVE, 900);  // hand
    calibrate_signal(AD, AD);
//    AD5270_Set(CHIP_SEL_DRIVE, 594);
//    AD5270_Set(CHIP_SEL_MEAS, 23);
//    AD5270_Shutdown(CHIP_SEL_DRIVE, 1);
//    AD5270_Shutdown(CHIP_SEL_MEAS, 1);

    mux_write(CHIP_SEL_MUX_SRC, elec_to_mux[0], MUX_EN);
    mux_write(CHIP_SEL_MUX_SINK, elec_to_mux[1], MUX_EN);
    mux_write(CHIP_SEL_MUX_VP, elec_to_mux[0], MUX_EN);
    mux_write(CHIP_SEL_MUX_VN, elec_to_mux[1], MUX_EN);

    // Serial.print("Current gain: ");
    // Serial.println(current_gain);
    // Serial.print("Measurement gain: ");
    // Serial.println(voltage_gain);
    // Serial.print("Sample rate (uS per reading): ");
    // Serial.println(sample_rate, 4);
    // Serial.print("Samples per period: ");
    // Serial.println(samples_per_period);
    // Serial.print("Reference signal magnitude (V): ");
    // Serial.println(ref_signal_mag, 4);
    // Serial.print("Reference signal phase offset (radians): ");
    // Serial.println(phase_offset, 4);

    uint16_t i;

    // /* Read resting impedance state for calibration */
    // for(i = 0; i < 30; i++)
    // {
    //     read_frame(AD, AD, signal_rms, signal_mag, signal_phase, NUM_ELECTRODES);
        
    //     uint16_t j;
    //     for (j = 0; j < NUM_MEAS; j++)
    //     {
    //         if (signal_rms[j] != 0)
    //             cur_frame[j] = 0.80 * cur_frame[j] + 0.20 * (signal_rms[j]);
    //     }
    // }

    // Serial.println("origin frame");
    // for (i = 0; i < NUM_MEAS; i++)
    // {                                                                             
    //     Serial.println(cur_frame[i], 4);
    // }
}

void loop() 
{   
    uint16_t i;
    
//    double mag, rms;
//    uint16_t error;
//    read_signal(&rms, &mag, NULL, &error, 0);
//    Serial.print(rms,4);
//    Serial.print("\t");
//    Serial.println(mag, 4);
//    delay(1000);
    
    // inject current at one electrode (HIGH)
    // drain current through an adjacent electrode (GND)
    // measure voltage at all adjacent pairs around the rectangle
    // one row of 32 readings corresponds to the same current injection
    read_frame(AD, AD, signal_rms, signal_mag, signal_phase, NUM_ELECTRODES);

//    for(i = 0; i < 20; i++)
//    {
//        cur_frame[i] = 0.50 * cur_frame[i] + 0.50 * signal_rms[i];
//        Serial.print(cur_frame[i], 4);
//        Serial.print("\t");
//    }
//    Serial.println();

    if (millis() - frame_delay > 350) {
        // Serial.println("frame");
        for (i = 0; i < NUM_MEAS; i++)
        // (int i: values_to_send) // to send all, use: `for (i = 0; i < NUM_MEAS; i++) {...}`
        {       
            //if (signal_rms[i] != 0)
            //    cur_frame[i] = 0.50 * cur_frame[i] + 0.50 * (signal_rms[i]);
            // Serial.println(cur_frame[i], 4);
            Serial.print(signal_rms[i], 4);  // cur_frame[i]
            Serial.print(", ");
        }
        Serial.print("\n");
//        frame_delay_prev = frame_delay;
        frame_delay = millis();
//        Serial.println(frame_delay - frame_delay_prev);
        delay(10);
    }

//    if (Serial.available())
//    {
//        uint8_t key = Serial.read();
////
////        mux_write(CHIP_SEL_MUX_VP, elec_to_mux[pin_num], MUX_EN);
////        //mux_write(CHIP_SEL_MUX_VN, elec_to_mux[(pin_num + NUM_ELECTRODES/2) % NUM_ELECTRODES], MUX_EN);
////        mux_write(CHIP_SEL_MUX_VN, elec_to_mux[(pin_num + 1) % NUM_ELECTRODES], MUX_EN);
////        Serial.print(pin_num);
////        Serial.print("\t");
////        //Serial.println((pin_num + NUM_ELECTRODES/2) % NUM_ELECTRODES);
////        Serial.println((pin_num + 1) % NUM_ELECTRODES);
////        pin_num = (pin_num+1) % NUM_ELECTRODES;
//
////        mux_write(CHIP_SEL_MUX_SRC, elec_to_mux[pin_num], MUX_EN);
////        Serial.println(pin_num);
////        pin_num = (pin_num+1) % NUM_ELECTRODES;
//
//        const int rheo = CHIP_SEL_DRIVE;
//        if (rheo_val >= 0x400) {
//            AD5270_Shutdown(rheo, 1);
//            Serial.println("off");     
//            rheo_val = 0;
//        } else {
//            if (rheo_val == 0)
//                AD5270_Shutdown(rheo, 0);
//            AD5270_Set(rheo, rheo_val);
//            Serial.println(rheo_val);
//            rheo_val++;
//        }
//    }
}
