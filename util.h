enum {STOP,LEFT,RIGHT,STRAIGHT};

void smooth_set_motors(uint8_t ccw, uint8_t cw) ;
void motion(uint8_t type);
float clipf(float f, float min, float max);


