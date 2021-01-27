#pragma once
/*======================================================================
* Author   : Haiming Zhang
* Email    : zhanghm_1995@qq.com
* Version  :　2020年12月14日
* Copyright    :
* Descriptoin  : Using C++ to do file related operation
* References   : https://github.com/Amerge/natsort
======================================================================*/

namespace cpp_common {

#include <cstddef>	/* size_t */
#include <cctype>
#include <string>

typedef char nat_char;

int strnatcmp(nat_char const *a, nat_char const *b);
int strnatcasecmp(nat_char const *a, nat_char const *b);

static inline int nat_isdigit(nat_char a){
    return isdigit((unsigned char) a);
}


static inline int nat_isspace(nat_char a){
    return isspace((unsigned char) a);
}


static inline nat_char nat_toupper(nat_char a){
    return toupper((unsigned char) a);
}


static int compare_right(nat_char const *a, nat_char const *b){
     int bias = 0;
     
     /* The longest run of digits wins.  That aside, the greatest
	value wins, but we can't know that it will until we've scanned
	both numbers to know that they have the same magnitude, so we
	remember it in BIAS. */
     for (;; a++, b++) {
	  if (!nat_isdigit(*a)  &&  !nat_isdigit(*b))
	       return bias;
	  if (!nat_isdigit(*a))
	       return -1;
	  if (!nat_isdigit(*b))
	       return +1;
	  if (*a < *b) {
	       if (!bias)
		    bias = -1;
	  } else if (*a > *b) {
	       if (!bias)
		    bias = +1;
	  } else if (!*a  &&  !*b)
	       return bias;
     }

     return 0;
}


static int compare_left(nat_char const *a, nat_char const *b){
     /* Compare two left-aligned numbers: the first to have a
        different value wins. */
     for (;; a++, b++) {
	  if (!nat_isdigit(*a)  &&  !nat_isdigit(*b))
	       return 0;
	  if (!nat_isdigit(*a))
	       return -1;
	  if (!nat_isdigit(*b))
	       return +1;
	  if (*a < *b)
	       return -1;
	  if (*a > *b)
	       return +1;
     }

     return 0;
}


static int strnatcmp0(nat_char const *a, nat_char const *b, int fold_case){
     int ai, bi;
     nat_char ca, cb;
     int fractional, result;

     ai = bi = 0;
     while (1) {
	  ca = a[ai]; cb = b[bi];

	  /* skip over leading spaces or zeros */
	  while (nat_isspace(ca))
	       ca = a[++ai];

	  while (nat_isspace(cb))
	       cb = b[++bi];

	  /* process run of digits */
	  if (nat_isdigit(ca)  &&  nat_isdigit(cb)) {
	       fractional = (ca == '0' || cb == '0');

	       if (fractional) {
		    if ((result = compare_left(a+ai, b+bi)) != 0)
			 return result;
	       } else {
		    if ((result = compare_right(a+ai, b+bi)) != 0)
			 return result;
	       }
	  }

	  if (!ca && !cb) {
	       /* The strings compare the same.  Perhaps the caller
                  will want to call strcmp to break the tie. */
	       return 0;
	  }

	  if (fold_case) {
	       ca = nat_toupper(ca);
	       cb = nat_toupper(cb);
	  }

	  if (ca < cb)
	       return -1;

	  if (ca > cb)
	       return +1;

	  ++ai; ++bi;
     }
}

/*The following doesn't consider case of string*/
int strnatcmp(nat_char const *a, nat_char const *b) {
     return strnatcmp0(a, b, 0);
}


/* Compare, recognizing numeric string and ignoring case.
 * This one is the general case where lower-case and upper-
 * case are considered as same.
 *  */
int strnatcasecmp(nat_char const *a, nat_char const *b) {
     return strnatcmp0(a, b, 1);
}

//You can use this function with std::sort and vector
bool compareNat(const std::string& a,const std::string& b){
    if(strnatcasecmp(a.c_str(),b.c_str())==-1) return true;
    else return false;
    }

//You can use this function with qsort
int compareNatq(const void *a, const void *b){
    char const *pa = *(char const **)a, *pb = *(char const **)b;
    return strnatcasecmp(pa, pb);
}

} // namespace cpp_common