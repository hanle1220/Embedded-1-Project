

#ifndef ODOMETER_H_
#define ODOMETER_H_

void setEncoderPosition(uint8_t id, int32_t position);
int32_t getEncoderPosition(uint8_t id);
void selectEncoderIncMode(uint8_t id);
void selectEncoderDecMode(uint8_t id);

#endif /* ODOMETER_H_ */
