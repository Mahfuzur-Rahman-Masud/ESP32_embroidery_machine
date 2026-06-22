#ifndef CUSTOM_COMMANDS_H_
#define CUSTOM_COMMANDS_H_

#include "grbl/core_handlers.h"

// Expose your initialization function so driver.c/main.c can call it
void custom_commands_init(void);

#endif /* CUSTOM_COMMANDS_H_ */