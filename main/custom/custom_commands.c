#include "custom_commands.h"
#include <string.h>

static on_user_command_ptr on_user_command_v1 = NULL;

static status_code_t custom_commands_exec(char* line)
{

    report_message("CCMD: ", Message_Info);
    report_message( line, Message_Info);

    if (strcasecmp(line, "abort") == 0) {
        report_message("CCMD: ABORT", Message_Info);
        grbl.enqueue_realtime_command(CMD_STOP);
        return Status_OK;
    }


    if(on_user_command_v1 != NULL){
        return on_user_command_v1(line);
    }


    return Status_Unhandled;
}


void custom_commands_init(void)
{
    on_user_command_v1 = grbl.on_user_command;
    grbl.on_user_command = custom_commands_exec;

}