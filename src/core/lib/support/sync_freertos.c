
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

#include <grpc/support/port_platform.h>

#ifdef GPR_FREERTOS_SYNC

#include <errno.h>
#include <grpc/support/log.h>
#include <grpc/support/sync.h>
#include <grpc/support/time.h>
#include <time.h>
#include "src/core/lib/profiling/timers.h"


#include "FreeRTOS.h"
#include "semphr.h"

// NOTE: Versions 8+ of FreeRTOS require different macro names
// Refer to FreeRTOS.h

void gpr_mu_init(gpr_mu* mu)
{
    mu->lock = (gpr_mu)xSemaphoreCreateMutex();
    GPR_ASSERT(mu->lock != NULL);
}

void gpr_mu_destroy(gpr_mu* mu)
{
#ifdef xSemaphoreDelete
    xSemaphoreDelete((xSemaphoreHandle)mu->lock);
#endif
}

void gpr_mu_lock(gpr_mu* mu) {
  GPR_TIMER_BEGIN("gpr_mu_lock", 0);
  GPR_ASSERT(xSemaphoreTake((xSemaphoreHandle)mu->lock, portMAX_DELAY) == pdTRUE);
  GPR_TIMER_END("gpr_mu_lock", 0);
}

void gpr_mu_unlock(gpr_mu* mu) {
  GPR_TIMER_BEGIN("gpr_mu_unlock", 0);
  GPR_ASSERT(xSemaphoreGive((xSemaphoreHandle)mu->lock) == pdTRUE);
  GPR_TIMER_END("gpr_mu_unlock", 0);
}

int gpr_mu_trylock(gpr_mu* mu) {
  int err;

  GPR_TIMER_BEGIN("gpr_mu_trylock", 0);
  err = xSemaphoreTake((xSemaphoreHandle)mu->lock, (portTickType)0);
  GPR_TIMER_END("gpr_mu_trylock", 0);

  return err == 0;
}

/*----------------------------------------*/

static void gpr_cv_append_mu(gpr_cv* cv, gpr_mu* mu)
{
    // Append to front
    mu->next = cv->next; 
    cv->next = &mu->lock;
}

static void gpr_cv_remove_mu(gpr_cv* cv, gpr_mu* mu)
{
    gpr_mu * curr;
    gpr_mu * prev;

    curr = cv->next;

    while (curr != NULL)
    {
        if (curr == mu)
        {
            if (curr == cv->next)
            {
                cv->next = curr->next;
            }
            else
            {
                prev->next = curr->next;
            }
            break;
        }

        prev = curr;
        curr = curr->next;
    }
}

// Give from end - FIFO order
static gpr_mu * gpr_cv_give_next(gpr_cv* cv)
{
    gpr_mu * curr;

    curr = cv->next;

    if (curr != NULL)
    {
        while (curr->next != NULL)
        {
            curr = curr->next;
        }

        xSemaphoreGive((xSemaphoreHandle)curr->lock);
    }

    return curr;
}

void gpr_cv_init(gpr_cv* cv)
{
    cv->next = NULL;
    gpr_mu_init(&cv->lock);
}

void gpr_cv_destroy(gpr_cv* cv)
{
//    gpr_mu_lock(&cv->lock);
    GPR_ASSERT(cv->semphrs == NULL);
//    gpr_mu_unlock(&cv->lock);
    gpr_mu_destroy(&cv->lock);
}

// Precondition: mu has been initialized
int gpr_cv_wait(gpr_cv* cv, gpr_mu* mu, gpr_timespec abs_deadline)
{
    int err;
    portTickType tickCount;

    tickCount = (portTickType)abs_deadline;
    //tickCount = (portTickType)(((uint64_t)abs_deadline) * configTICK_RATE_HZ ) / 1000;

    gpr_mu_lock(&cv->lock);
    gpr_cv_append_mu(cv, mu);
    gpr_mu_unlock(&cv->lock);

    // If a higher-priority task calls signal() or broadcast() before this semaphore
    // is taken, xSemaphoreTake should not block!!
    err = xSemaphoreTake((xSemaphoreHandle)mu->lock, tickCount);

    gpr_mu_lock(&cv->lock);
    gpr_cv_remove_mu(cv, mu);
    gpr_mu_unlock(&cv->lock);

    // Return non-zero if expired before signal() or broadcast() called
    return err == pdFALSE ? 1 : 0;
}

void gpr_cv_signal(gpr_cv* cv)
{
    gpr_mu_lock(&cv->lock);
    gpr_give_next(cv);
    gpr_mu_unlock(&cv->lock);
}

void gpr_cv_broadcast(gpr_cv* cv)
{
    gpr_mu_lock(&cv->lock);
    while (gpr_give_next(cv) != NULL);
    gpr_mu_unlock(&cv->lock);
}

/*----------------------------------------*/

void gpr_once_init(gpr_once* once, void (*init_function)(void)) {
    // TODO: this - "once" is supposed to be initialized with a macro
    // we provide before this function is called, otherwise undefined
    // behavior. Struct with lock and counter?
}

#endif /* GPR_FREERTOS_SYNC */
