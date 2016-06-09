uint8 IR_CAL_CODE_FINISH = 0     #Sent by FMU, indicates that calibration process is complete
uint8 IR_CAL_CODE_START = 1      # Sent by GCS, indicates that calibration process should begin
uint8 IR_CAL_CODE_SAMPLE = 2     #Sent by FMU, indicates that the data varaible contains the ADC sample average produced by the calibration routine for a certain range measurement
uint8 IR_CAL_CODE_SAMPLE_VAR = 3 #Sent by FMU, indicates that the data variable contains the sample variance produced by the caliberation routine for a certain range measurement
uint8 IR_CAL_CODE_EST_COEFF = 4  #Sent by FMU, indicates that the data variable contains an estmiator coefficient produced by the calibration routine for a certain term in the estimator

uint8 data_code #Code from IR_CAL_CODE_ENUM that puts the data into context.
float32 data      #Variable for exchanging sensor samples or estimator coefficients between GCS and FMU.
