
#ifndef IR_ALIGNER_MANUAL_H
#define IR_ALIGNER_MANUAL_H

namespace IRAlignerManual
{

void begin();
void start();
void update();
void stop();

bool isFinished();
bool hasFailed();
bool isDone();

const char* getStateName();
const char* getDebugStatus();

} // namespace IRAlignerManual

#endif // IR_ALIGNER_MANUAL_H

