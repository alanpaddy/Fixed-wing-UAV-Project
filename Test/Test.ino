void loop(){
                  tempValue = getTemp(&tempSensor);
                  if (is_tuning){                                                             // Check whether tunner is turned on
                      LCD.setCursor(0, 0);                                                     // Set the cursor to the 16 - position to print
                      LCD.print("TUNING");                                                        // TUNE
                      val = 0;
                      val = (aTune.Runtime());                                                // Run the Auto tunner and check whether it is finished the tuning
                      if(val != 0){                                                             // If tuning is done
                        is_tuning = 0;                                                          // Turn off the tunner
                        kp = aTune.GetKp();                                               // Get the constants from the auto tunner module
                        ki = aTune.GetKi();
                        kd = aTune.GetKd();
                        PID.SetTunings(kp,ki,kd);                         // Set the tunned constants to the PID
                        PID.SetMode(AUTOMATIC);
                     }
                  }else{
                     PID.Compute();
                  }
         }
