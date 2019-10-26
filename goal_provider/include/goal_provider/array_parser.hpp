#ifndef _ARRAY_PARSER_HPP
#define _ARRAY_PARSER_HPP

#include <cstdio>  // for EOF
#include <string>
#include <sstream>
#include <vector>

class ArrayParser {
    public:
    ArrayParser();
    ~ArrayParser();
    /** @brief Parse a vector of vector of floats from a string.
      * @param input
      * @param error_return
      * Syntax is [[1.0, 2.0], [3.3, 4.4, 5.5], ...] 
    **/
    std::vector<std::vector<float> > parseVVF(const std::string& input, std::string& error_return);
};


#endif /* !_ARRAY_PARSER_HPP */