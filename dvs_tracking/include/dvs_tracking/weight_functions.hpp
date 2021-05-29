#ifndef WEIGHT_FUNCTIONS_HPP
#define WEIGHT_FUNCTIONS_HPP

namespace weight_functions {

#define WEIGHT_FUNCTIONS_TUKEY_B_SQUARE 21.95016201  // b = 4.6851
/**
 * Tukey norm
 * @param error arguments of the norm
 *
 * @return Tukey norm evaluated at error
 */
float Tukey(const float error) {
    const float x_square = error * error;
    if (x_square <= WEIGHT_FUNCTIONS_TUKEY_B_SQUARE) {
        const float tmp = 1.0f - x_square / WEIGHT_FUNCTIONS_TUKEY_B_SQUARE;
        return tmp * tmp;
    } else {
        return 0.0f;
    }
}

#define WEIGHT_FUNCTIONS_HUBER_K 1.345
/**
 * Huber norm
 * @param error arguments of the norm
 *
 * @return Huber norm evaluated at error
 */
float Huber(const float error) {
    const float abs_error = std::fabs(error);
    return (abs_error < WEIGHT_FUNCTIONS_HUBER_K)
               ? 1.0f
               : WEIGHT_FUNCTIONS_HUBER_K / abs_error;
}

}  // namespace weight_functions

#endif  // WEIGHT_FUNCTIONS_HPP
