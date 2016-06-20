uint8 IR_CAL_CODE_FINISH = 1     #Sent by FMU, indicates that calibration process is complete
uint8 IR_CAL_CODE_START = 2      # Sent by GCS, indicates that calibration process should begin
uint8 IR_CAL_CODE_SAMPLE = 3     #Sent by FMU, indicates that the data varaible contains the ADC sample average produced by the calibration routine for a certain range measurement
uint8 IR_CAL_CODE_SAMPLE_VAR = 4 #Sent by FMU, indicates that the data variable contains the sample variance produced by the caliberation routine for a certain range measurement
uint8 IR_CAL_CODE_EST_COEFF = 5  #Sent by FMU, indicates that the data variable contains an estmiator coefficient produced by the calibration routine for a certain term in the estimator

uint64 timestamp
uint8 data_code   #Code from IR_CAL_CODE_ENUM that puts the data into context.
float32 data      #Variable for exchanging sensor samples or estimator coefficients between GCS and FMU.
