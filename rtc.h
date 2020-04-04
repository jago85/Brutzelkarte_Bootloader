
#ifndef RTC_H
#define RTC_H

#include <time.h>

int rtc_present(void);
void rtc_read(struct tm * time);
void rtc_write(struct tm * time);

#endif
