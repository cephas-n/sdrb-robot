#ifndef _UTILITIES_H_
#define _UTILITIES_H_

/**
 * @brief Generates the next lexicographically greater permutation of a sequence. 
 * Rearranges the elements in the range [first, last) into the next lexicographically greater
 * permutation if it exists, or reorders the range to the first permutation if the current
 * permutation is the lexicographically largest.
 * The function follows the lexicographical order, meaning each subsequent permutation generated
 * will be greater than the previous one. If the range is already in the lexicographically largest permutation, the function will return false and the range will be rearranged to the first permutation.
 * 
 * @param first An iterator pointing to the beginning of the range.
 * @param last An iterator pointing to the end of the range.
 * @return true if the next permutation was generated, false if the range was rearranged to the first permutation.
 */
template <typename BidirIt>
bool next_permutation(BidirIt first, BidirIt last) {
    if (first == last)
        return false;

    BidirIt i = last;
    if (first == --i)
        return false;

    while (true) {
        BidirIt i1, i2;

        i1 = i;

        if (*--i < *i1) {
            i2 = last;
            while (!(*i < *--i2))
                ;

            // Swap elements manually
            auto temp = *i;
            *i = *i2;
            *i2 = temp;

            // Reverse the suffix manually
            BidirIt it1 = i1;
            BidirIt it2 = last;
            while (it1 < it2) {
                temp = *it1;
                *it1 = *--it2;
                *it2 = temp;
                ++it1;
            }

            return true;
        }

        if (i == first) {
            // Reverse the entire range manually
            BidirIt it1 = first;
            BidirIt it2 = last;
            while (it1 < it2) {
                auto temp = *it1;
                *it1 = *--it2;
                *it2 = temp;
                ++it1;
            }
            return false;
        }
    }
}

//
#endif