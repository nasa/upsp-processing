#include <algorithm>
#include <iterator>
#include <sstream>
#include <string>
#include <vector>

#include "utils/general_utils.h"

/*****************************************************************************/
unsigned int factorial(unsigned int n) {
    if (n == 0) return 1;
    else return n * factorial(n-1);
}

/*****************************************************************************/
void split_string(std::string str, char delim, std::vector<std::string>& terms) {
    std::stringstream ss(str);

    terms.clear();
    std::string segment;
    while(getline(ss, segment, delim)) {
        terms.push_back(segment);
    }
} 

/*****************************************************************************/
void split_whitespace(std::string str, std::vector<std::string>& terms) {
    terms.clear();
    std::istringstream buf(str);

    std::copy(std::istream_iterator<std::string>(buf), 
            std::istream_iterator<std::string>(), std::back_inserter(terms));
}

/*****************************************************************************/
int round_up(int input, int multiple) {

    if (multiple == 0) {
        return input;
    }

    int remainder = std::abs(input) % multiple;
    if (remainder == 0) {
        return input;
    }

    if (input < 0) {
        return -(std::abs(input) - remainder);
    } else {
        return input + multiple - remainder;
    }
}

/*****************************************************************************/
int round_down(int input, int multiple) {

    if (multiple == 0) {
        return input;
    }

    int remainder = std::abs(input) % multiple;
    if (remainder == 0) {
        return input;
    }

    if (input < 0) {
        return -(std::abs(input) + multiple - remainder);
    } else {
        return input - remainder;
    }
}

/*****************************************************************************/
unsigned int round_down(unsigned int input, unsigned int multiple) {

    if (multiple == 0) {
        return input;
    }

    unsigned int remainder = input % multiple;
    if (remainder == 0) {
        return input;
    }

    return input - remainder;
}
