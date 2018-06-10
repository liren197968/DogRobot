#ifndef __SPEECH_H
#define __SPEECH_H

#include <stdint.h>

void Speech_Init(void);
void StandBySpeech_Play(void);
void ForwardSpeech_Play(void);
void BackwrdSpeech_Play(void);
void ClckWisSpeech_Play(void);
void AtiClckSpeech_Play(void);
void TrasfrmSpeech_Play(void);
void FrdRollSpeech_Play(void);
void BakRollSpeech_Play(void);
void GestureSpeech_Play(void);
void TxtUpdaSpeech_Play(void);
void Mp3SpeechStop_Play(void);

extern uint8_t     SpeechReturnData;

#endif
