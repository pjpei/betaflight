/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#define SRC_MAIN_SCHEDULER_C_

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#include "drivers/accgyro/accgyro.h"

#include "build/build_config.h"
#include "build/debug.h"

#include "common/maths.h"
#include "common/time.h"
#include "common/utils.h"

#include "drivers/time.h"
#include "drivers/accgyro/accgyro.h"
#include "drivers/system.h"

#include "fc/core.h"
#include "fc/tasks.h"

#include "rx/rx.h"
#include "flight/failsafe.h"

#include "scheduler.h"

#include "sensors/gyro_init.h"

// DEBUG_SCHEDULER, timings for:
// 0 - Average time spent executing check function
// 1 - Time spent priortising
// 2 - time spent in scheduler

// DEBUG_SCHEDULER_DETERMINISM, requires USE_LATE_TASK_STATISTICS to be defined
// 0 - Gyro task start cycle time in 10th of a us
// 1 - ID of late task
// 2 - Amount task is late in 10th of a us
// 3 - Gyro lock skew in 10th of a us

// DEBUG_TIMING_ACCURACY, requires USE_LATE_TASK_STATISTICS to be defined
// 0 - % CPU busy
// 1 - Tasks late in last second
// 2 - Total lateness in last second in 10ths us
// 3 - Total tasks run in last second

extern task_t tasks[];

static FAST_DATA_ZERO_INIT task_t *currentTask = NULL;
static FAST_DATA_ZERO_INIT bool ignoreCurrentTaskExecRate;
static FAST_DATA_ZERO_INIT bool ignoreCurrentTaskExecTime;

int32_t schedLoopStartCycles;
static int32_t schedLoopStartMinCycles;
static int32_t schedLoopStartMaxCycles;
static uint32_t schedLoopStartDeltaDownCycles;
static uint32_t schedLoopStartDeltaUpCycles;

int32_t taskGuardCycles;
static int32_t taskGuardMinCycles;
static int32_t taskGuardMaxCycles;
static uint32_t taskGuardDeltaDownCycles;
static uint32_t taskGuardDeltaUpCycles;

FAST_DATA_ZERO_INIT uint16_t averageSystemLoadPercent = 0;

static FAST_DATA_ZERO_INIT int taskQueuePos = 0;
STATIC_UNIT_TESTED FAST_DATA_ZERO_INIT int taskQueueSize = 0;

static FAST_DATA_ZERO_INIT bool gyroEnabled;

static int32_t desiredPeriodCycles;
static uint32_t lastTargetCycles;

static uint8_t skippedRxAttempts = 0;
#ifdef USE_OSD
static uint8_t skippedOSDAttempts = 0;
#endif

#if defined(USE_LATE_TASK_STATISTICS)
static int16_t lateTaskCount = 0;
static uint32_t lateTaskTotal = 0;
static int16_t taskCount = 0;
static uint32_t nextTimingCycles;
#endif

static timeMs_t lastFailsafeCheckMs = 0;

// No need for a linked list for the queue, since items are only inserted at startup

STATIC_UNIT_TESTED FAST_DATA_ZERO_INIT task_t* taskQueueArray[TASK_COUNT + 1]; // extra item for NULL pointer at end of queue

void queueClear(void)
{
    memset(taskQueueArray, 0, sizeof(taskQueueArray));
    taskQueuePos = 0;
    taskQueueSize = 0;
}

bool queueContains(task_t *task)
{
    for (int ii = 0; ii < taskQueueSize; ++ii) {
        if (taskQueueArray[ii] == task) {
            return true;
        }
    }
    return false;
}

bool queueAdd(task_t *task)
{
    if ((taskQueueSize >= TASK_COUNT) || queueContains(task)) {
        return false;
    }
    for (int ii = 0; ii <= taskQueueSize; ++ii) {
        if (taskQueueArray[ii] == NULL || taskQueueArray[ii]->staticPriority < task->staticPriority) {
            memmove(&taskQueueArray[ii+1], &taskQueueArray[ii], sizeof(task) * (taskQueueSize - ii));
            taskQueueArray[ii] = task;
            ++taskQueueSize;
            return true;
        }
    }
    return false;
}

bool queueRemove(task_t *task)
{
    for (int ii = 0; ii < taskQueueSize; ++ii) {
        if (taskQueueArray[ii] == task) {
            memmove(&taskQueueArray[ii], &taskQueueArray[ii+1], sizeof(task) * (taskQueueSize - ii));
            --taskQueueSize;
            return true;
        }
    }
    return false;
}

/*
 * Returns first item queue or NULL if queue empty
 */
FAST_CODE task_t *queueFirst(void)
{
    taskQueuePos = 0;
    return taskQueueArray[0]; // guaranteed to be NULL if queue is empty
}

/*
 * Returns next item in queue or NULL if at end of queue
 */
FAST_CODE task_t *queueNext(void)
{
    return taskQueueArray[++taskQueuePos]; // guaranteed to be NULL at end of queue
}

static timeUs_t taskTotalExecutionTime = 0;
static FAST_DATA_ZERO_INIT uint32_t totalWaitingTasks;
static FAST_DATA_ZERO_INIT uint32_t totalWaitingTasksSamples;

void taskSystemLoad(timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);

    // Calculate system load
    if (totalWaitingTasksSamples > 0) {
        averageSystemLoadPercent = 100 * totalWaitingTasks / totalWaitingTasksSamples;
        totalWaitingTasksSamples = 0;
        totalWaitingTasks = 0;
    }
#if defined(SIMULATOR_BUILD)
    averageSystemLoadPercent = 0;
#endif
}

timeUs_t checkFuncMaxExecutionTimeUs;
timeUs_t checkFuncTotalExecutionTimeUs;
timeUs_t checkFuncMovingSumExecutionTimeUs;
timeUs_t checkFuncMovingSumDeltaTimeUs;

void getCheckFuncInfo(cfCheckFuncInfo_t *checkFuncInfo)
{
    checkFuncInfo->maxExecutionTimeUs = checkFuncMaxExecutionTimeUs;
    checkFuncInfo->totalExecutionTimeUs = checkFuncTotalExecutionTimeUs;
    checkFuncInfo->averageExecutionTimeUs = checkFuncMovingSumExecutionTimeUs / TASK_STATS_MOVING_SUM_COUNT;
    checkFuncInfo->averageDeltaTimeUs = checkFuncMovingSumDeltaTimeUs / TASK_STATS_MOVING_SUM_COUNT;
}

void getTaskInfo(taskId_e taskId, taskInfo_t * taskInfo)
{
    taskInfo->isEnabled = queueContains(getTask(taskId));
    taskInfo->desiredPeriodUs = getTask(taskId)->desiredPeriodUs;
    taskInfo->staticPriority = getTask(taskId)->staticPriority;
    taskInfo->taskName = getTask(taskId)->taskName;
    taskInfo->subTaskName = getTask(taskId)->subTaskName;
    taskInfo->maxExecutionTimeUs = getTask(taskId)->maxExecutionTimeUs;
    taskInfo->totalExecutionTimeUs = getTask(taskId)->totalExecutionTimeUs;
    taskInfo->averageExecutionTimeUs = getTask(taskId)->movingSumExecutionTimeUs / TASK_STATS_MOVING_SUM_COUNT;
    taskInfo->averageDeltaTimeUs = getTask(taskId)->movingSumDeltaTimeUs / TASK_STATS_MOVING_SUM_COUNT;
    taskInfo->latestDeltaTimeUs = getTask(taskId)->taskLatestDeltaTimeUs;
    taskInfo->movingAverageCycleTimeUs = getTask(taskId)->movingAverageCycleTimeUs;
#if defined(USE_LATE_TASK_STATISTICS)
    taskInfo->lateCount = getTask(taskId)->lateCount;
    taskInfo->runCount = getTask(taskId)->runCount;
    taskInfo->execTime = getTask(taskId)->execTime;
#endif
}

void rescheduleTask(taskId_e taskId, timeDelta_t newPeriodUs)
{
    task_t *task;

    if (taskId == TASK_SELF) {
        task = currentTask;
    } else if (taskId < TASK_COUNT) {
        task = getTask(taskId);
    } else {
        return;
    }
    task->desiredPeriodUs = MAX(SCHEDULER_DELAY_LIMIT, newPeriodUs);  // Limit delay to 100us (10 kHz) to prevent scheduler clogging

    // Catch the case where the gyro loop is adjusted
    if (taskId == TASK_GYRO) {
        desiredPeriodCycles = (int32_t)clockMicrosToCycles((uint32_t)getTask(TASK_GYRO)->desiredPeriodUs);
    }
}

void setTaskEnabled(taskId_e taskId, bool enabled)
{
    if (taskId == TASK_SELF || taskId < TASK_COUNT) {
        task_t *task = taskId == TASK_SELF ? currentTask : getTask(taskId);
        if (enabled && task->taskFunc) {
            queueAdd(task);
        } else {
            queueRemove(task);
        }
    }
}

timeDelta_t getTaskDeltaTimeUs(taskId_e taskId)
{
    if (taskId == TASK_SELF) {
        return currentTask->taskLatestDeltaTimeUs;
    } else if (taskId < TASK_COUNT) {
        return getTask(taskId)->taskLatestDeltaTimeUs;
    } else {
        return 0;
    }
}

// Called by tasks executing what are known to be short states
void schedulerIgnoreTaskStateTime()
{
    ignoreCurrentTaskExecRate = true;
    ignoreCurrentTaskExecTime = true;
}

// Called by tasks with state machines to only count one state as determining rate
void schedulerIgnoreTaskExecRate()
{
    ignoreCurrentTaskExecRate = true;
}

// Called by tasks without state machines executing in what is known to be a shorter time than peak
void schedulerIgnoreTaskExecTime()
{
    ignoreCurrentTaskExecTime = true;
}

bool schedulerGetIgnoreTaskExecTime()
{
    return ignoreCurrentTaskExecTime;
}

void schedulerResetTaskStatistics(taskId_e taskId)
{
    if (taskId == TASK_SELF) {
        currentTask->anticipatedExecutionTime = 0;
        currentTask->movingSumDeltaTimeUs = 0;
        currentTask->totalExecutionTimeUs = 0;
        currentTask->maxExecutionTimeUs = 0;
    } else if (taskId < TASK_COUNT) {
        getTask(taskId)->anticipatedExecutionTime = 0;
        getTask(taskId)->movingSumDeltaTimeUs = 0;
        getTask(taskId)->totalExecutionTimeUs = 0;
        getTask(taskId)->maxExecutionTimeUs = 0;
    }
}

void schedulerResetTaskMaxExecutionTime(taskId_e taskId)
{
    if (taskId == TASK_SELF) {
        currentTask->maxExecutionTimeUs = 0;
    } else if (taskId < TASK_COUNT) {
        task_t *task = getTask(taskId);
        task->maxExecutionTimeUs = 0;
#if defined(USE_LATE_TASK_STATISTICS)
        task->lateCount = 0;
        task->runCount = 0;
#endif
    }
}

void schedulerResetCheckFunctionMaxExecutionTime(void)
{
    checkFuncMaxExecutionTimeUs = 0;
}

static FAST_DATA_ZERO_INIT bool calculateTaskStatistics;

void schedulerInit(void)
{
    calculateTaskStatistics = true;
    queueClear();
    queueAdd(getTask(TASK_SYSTEM));
}

static timeDelta_t taskNextStateTime;

FAST_CODE void schedulerSetNextStateTime(timeDelta_t nextStateTime)
{
    taskNextStateTime = nextStateTime;
}

FAST_CODE timeDelta_t schedulerGetNextStateTime()
{
    return currentTask->anticipatedExecutionTime >> TASK_EXEC_TIME_SHIFT;
}

#if defined(UNIT_TEST)
task_t *unittest_scheduler_selectedTask;
uint8_t unittest_scheduler_selectedTaskDynamicPriority;

static void readSchedulerLocals(task_t *selectedTask, uint8_t selectedTaskDynamicPriority)
{
    unittest_scheduler_selectedTask = selectedTask;
    unittest_scheduler_selectedTaskDynamicPriority = selectedTaskDynamicPriority;
}
#endif

FAST_CODE void scheduler(void)
{
        // Cache currentTime
    const timeUs_t currentTimeUs = micros();
    // Check for realtime tasks
    bool outsideRealtimeGuardInterval = true;
    for (const task_t *task = queueFirst(); task != NULL && task->staticPriority == TASK_PRIORITY_REALTIME; task = queueNext()) {
        const timeUs_t nextExecuteAt = task->lastExecutedAtUs + task->desiredPeriodUs;
        if ((timeDelta_t)(currentTimeUs - nextExecuteAt) >= 0) {
            outsideRealtimeGuardInterval = false;
            break;
        }
    }
    // The task to be invoked
    task_t *selectedTask = NULL;
    uint16_t selectedTaskDynamicPriority = 0;
    // Update task dynamic priorities
    uint16_t waitingTasks = 0;
    for (task_t *task = queueFirst(); task != NULL; task = queueNext()) {
        // Task has checkFunc - event driven
        if (task->checkFunc) {
#if defined(SCHEDULER_DEBUG)
            const timeUs_t currentTimeBeforeCheckFuncCall = micros();
#else
            const timeUs_t currentTimeBeforeCheckFuncCall = currentTimeUs;
#endif
            // Increase priority for event driven tasks
            if (task->staticPriority == TASK_PRIORITY_TRIGGER) {
                if (task->checkFunc(currentTimeBeforeCheckFuncCall, currentTimeBeforeCheckFuncCall - task->lastExecutedAtUs)) {
                    task->taskAgeCycles = ((currentTimeUs - task->lastExecutedAtUs) / task->desiredPeriodUs);
                    if (task->taskAgeCycles > 0) {
                        task->dynamicPriority = 1 + task->staticPriority * task->taskAgeCycles;
                        waitingTasks++;
                    }
                } else {
                    task->taskAgeCycles = 0;
                }
            } else if (task->dynamicPriority > 0) {
                task->taskAgeCycles = 1 + ((currentTimeUs - task->lastSignaledAtUs) / task->desiredPeriodUs);
                task->dynamicPriority = 1 + task->staticPriority * task->taskAgeCycles;
                waitingTasks++;
            } else if (task->checkFunc(currentTimeBeforeCheckFuncCall, currentTimeBeforeCheckFuncCall - task->lastExecutedAtUs)) {
#if defined(SCHEDULER_DEBUG)
                DEBUG_SET(DEBUG_SCHEDULER, 3, micros() - currentTimeBeforeCheckFuncCall);
#endif
#ifndef SKIP_TASK_STATISTICS
                if (calculateTaskStatistics) {
                    const uint32_t checkFuncExecutionTime = micros() - currentTimeBeforeCheckFuncCall;
                    checkFuncMovingSumExecutionTimeUs += checkFuncExecutionTime - checkFuncMovingSumExecutionTimeUs / TASK_STATS_MOVING_SUM_COUNT;
                    checkFuncTotalExecutionTimeUs += checkFuncExecutionTime;   // time consumed by scheduler + task
                    checkFuncTotalExecutionTimeUs = MAX(checkFuncTotalExecutionTimeUs, checkFuncExecutionTime);
                }
#endif
                task->lastSignaledAtUs = currentTimeBeforeCheckFuncCall;
                task->taskAgeCycles = 1;
                task->dynamicPriority = 1 + task->staticPriority;
                waitingTasks++;
            } else {
                task->taskAgeCycles = 0;
            }
        } else {
            // Task is time-driven, dynamicPriority is last execution age (measured in desiredPeriodUss)
            // Task age is calculated from last execution
            task->taskAgeCycles = ((currentTimeUs - task->lastExecutedAtUs) / task->desiredPeriodUs);
            if (task->taskAgeCycles > 0) {
                task->dynamicPriority = 1 + task->staticPriority * task->taskAgeCycles;
                waitingTasks++;
            }
        }
        if (task->dynamicPriority > selectedTaskDynamicPriority) {
            const bool taskCanBeChosenForScheduling =
                (outsideRealtimeGuardInterval) ||
                (task->taskAgeCycles > 1) ||
                (task->staticPriority == TASK_PRIORITY_REALTIME);
            if (taskCanBeChosenForScheduling) {
                selectedTaskDynamicPriority = task->dynamicPriority;
                selectedTask = task;
            }
        }
    }
    totalWaitingTasksSamples++;
    totalWaitingTasks += waitingTasks;
    currentTask = selectedTask;
    if (selectedTask) {
        // Found a task that should be run
        selectedTask->taskLatestDeltaTimeUs = currentTimeUs - selectedTask->lastExecutedAtUs;
        selectedTask->lastExecutedAtUs = currentTimeUs;
        selectedTask->dynamicPriority = 0;
        // Execute task
#ifdef SKIP_TASK_STATISTICS
        selectedTask->taskFunc(currentTimeUs);
#else
        if (calculateTaskStatistics) {
            const timeUs_t currentTimeBeforeTaskCall = micros();
            selectedTask->taskFunc(currentTimeBeforeTaskCall);
            const timeUs_t taskExecutionTime = micros() - currentTimeBeforeTaskCall;
            selectedTask->movingSumExecutionTimeUs += taskExecutionTime - selectedTask->movingSumExecutionTimeUs / TASK_STATS_MOVING_SUM_COUNT;
            selectedTask->totalExecutionTimeUs += taskExecutionTime;   // time consumed by scheduler + task
            selectedTask->maxExecutionTimeUs = MAX(selectedTask->maxExecutionTimeUs, taskExecutionTime);
        } else {
            selectedTask->taskFunc(currentTimeUs);
        }
#endif
#if defined(SCHEDULER_DEBUG)
        DEBUG_SET(DEBUG_SCHEDULER, 2, micros() - currentTimeUs - taskExecutionTime); // time spent in scheduler
    } else {
        DEBUG_SET(DEBUG_SCHEDULER, 2, micros() - currentTimeUs);
#endif
    }
}

void schedulerEnableGyro(void)
{
    gyroEnabled = true;
}

uint16_t getAverageSystemLoadPercent(void)
{
    return averageSystemLoadPercent;
}
