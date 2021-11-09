/**
 * Date  : 25 Jul 2020
 * Author: Amjad Yousef Majid
 * Email : amjad.y.majid@gmail.com
 */

#ifndef SYSTEM_INCLUDE_TACHOMETER_H_
#define SYSTEM_INCLUDE_TACHOMETER_H_

<<<<<<< HEAD
/**
 * \brief specifies the direction of the motor rotation, relative to the front of the robot
 */
enum TachDirection{
  FORWARD, /**< Wheel is making robot move forward */
  STOPPED, /**< Wheel is stopped */
  REVERSE  /**< Wheel is making robot move backward */
};

/**
 * Initialize GPIO pins for input, which will be
 * used to determine the direction of rotation.
 * Initialize the input capture interface, which
 * will be used to measure the speed of rotation.
 * @param none
 * @return none
 * @brief  Initialize tachometer interface
 */
void tachometer_init(void(*userTaskLeft)(uint16_t time), void(*userTaskRight)(uint16_t time) );

/**
 * Get the most recent tachometer measurements.
 * @param leftTach is pointer to store last measured tachometer period of left wheel (units of 0.083 usec)
 * @param leftDir is pointer to store enumerated direction of last movement of left wheel
 * @param leftSteps is pointer to store total number of forward steps measured for left wheel (360 steps per ~220 mm circumference)
 * @param rightTach is pointer to store last measured tachometer period of right wheel (units of 0.083 usec)
 * @param rightDir is pointer to store enumerated direction of last movement of right wheel
 * @param rightSteps is pointer to store total number of forward steps measured for right wheel (360 steps per ~220 mm circumference)
 * @return none
 * @note Assumes Tachometer_Init() has been called<br>
 * @note Assumes Clock_Init48MHz() has been called
 * @brief Get the most recent tachometer measurement
=======
#define TACHOLEFT BIT5
#define TACHORIGHT BIT4

/** ------------tachometer_init------------
 * Setup the connections between the MCU and the tachometers using interrupts.
 * Inputs: userTaskLeft is the task to perform when the left tachometer gives an interrupt.
           userTaksRight is the task to perform when the right tachometer gives an interrupt.
 * Outputs: none
>>>>>>> 5a779fb5740b61a557ac1c5c4d18598159fae689
 */
void Tachometer_Get(uint16_t *leftTach, enum TachDirection *leftDir, int32_t *leftSteps,
                    uint16_t *rightTach, enum TachDirection *rightDir, int32_t *rightSteps);

#endif /* SYSTEM_INCLUDE_TACHOMETER_H_ */
