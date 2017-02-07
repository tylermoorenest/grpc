
/*
 *
 * Copyright 2015, Google Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 *     * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above
 * copyright notice, this list of conditions and the following disclaimer
 * in the documentation and/or other materials provided with the
 * distribution.
 *     * Neither the name of Google Inc. nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

/* FreeRTOS implementation for gpr threads. */

#include <grpc/support/port_platform.h>

#ifdef GPR_FREERTOS_SYNC

#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

static void global_entry(void *v)
{
    gpr_thd_id_t *task = (gpr_thd_id_t *)v;

    task->mTaskHandle = xTaskGetCurrentTaskHandle();
    vTaskSetApplicationTaskTag(task->mTaskHandle, (pdTASK_HOOK_CODE)task);

    (*task->mEntry)(task->mParams);
}


int gpr_thd_new(gpr_thd_id *t, void (*thd_body)(void *arg), void *arg,
                const gpr_thd_options *options) {
    // Need to define gpr_thd_id_t to contain all the requisite info for FreeRTOS
    // Tasks or co-routines
    // If "joinable", should it be a co-routine instead of a task?
    //
    return 0;
}

gpr_thd_id gpr_thd_currentid(void)
{
    gpr_thd_id  *retval = NULL;
    xTaskHandle task = xTaskGetCurrentTaskHandle();

    if (task != NULL)
    {
        retval = (gpr_thd_id *)xTaskGetApplicationTaskTag(task);
    }

    return *retval;
}

void gpr_thd_join(gpr_thd_id t)
{
    // No "Join" functionality for FreeRTOS
    // Can we use "co-routines" ?
}

int nl_task_create(nl_task_entry_point_t aEntry, const char *aName, void *aStack, size_t aStackSize, nl_task_priority_t aPriority, void *aParams, nl_task_t *aOutTask)
{
    int retval = NLER_ERROR_BAD_INPUT;

    NLER_ASSERT(((unsigned)aStack % NLER_REQUIRED_STACK_ALIGNMENT) == 0);

    if ((aOutTask != NULL) && (aName != NULL) && (aPriority < configMAX_PRIORITIES))
    {
        portBASE_TYPE   err;
        xTaskHandle     task;

        aOutTask->mName = aName;
        aOutTask->mStack = aStack;
        aOutTask->mStackSize = aStackSize;
        aOutTask->mPriority = aPriority;
        aOutTask->mParams = aParams;
        aOutTask->mNativeTask = (void *)aEntry;

        err = xTaskGenericCreate(global_entry,
                                 (const signed char *)aName,
                                 aStackSize / sizeof(portSTACK_TYPE),
                                 aOutTask,
                                 aPriority,
                                 &task,
                                 (portSTACK_TYPE *)aStack,
                                 NULL);

        if (err == pdPASS)
        {
            retval = NLER_SUCCESS;
        }
        else
        {
            retval = NLER_ERROR_NO_RESOURCE;
        }
    }

    if (retval != NLER_SUCCESS)
    {
        // Print error
    }

    return retval;
}

void nl_task_suspend(nl_task_t *aTask)
{
    if (aTask != NULL)
    {
        vTaskSuspend(aTask->mNativeTask);
    }
}

void nl_task_resume(nl_task_t *aTask)
{
    if (aTask != NULL)
    {
        vTaskResume(aTask->mNativeTask);
    }
}

void nl_task_set_priority(nl_task_t *aTask, int aPriority)
{
    if (aTask != NULL)
    {
        aTask->mPriority = aPriority;
        vTaskPrioritySet(aTask->mNativeTask, aPriority);
    }
}

nl_task_t *nl_task_get_current(void)
{
    nl_task_t  *retval = NULL;
    xTaskHandle task = xTaskGetCurrentTaskHandle();

    if (task != NULL)
    {
        retval = (nl_task_t *)xTaskGetApplicationTaskTag(task);
    }

    return retval;
}

void nl_task_sleep_ms(nl_time_ms_t aDurationMS)
{
    vTaskDelay(nl_time_ms_to_delay_time_native(aDurationMS));
}

void nl_task_yield(void)
{
    vTaskDelay(1);
}
#endif /* GPR_POSIX_SYNC */
