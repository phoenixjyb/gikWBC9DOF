//
// File: validatestring.cpp
//
// MATLAB Coder version            : 24.2
// C/C++ source code generated on  : 09-Oct-2025 12:02:50
//

// Include Files
#include "validatestring.h"
#include "gik9dof_codegen_inuse_solveGIKStepWrapper_data.h"
#include "rt_nonfinite.h"
#include <algorithm>
#include <cstring>

// Function Definitions
//
// Arguments    : const char str_data[]
//                const int str_size[2]
//                char out_data[]
//                int out_size[2]
// Return Type  : void
//
namespace gik9dof {
namespace coder {
void validatestring(const char str_data[], const int str_size[2],
                    char out_data[], int out_size[2])
{
  static const char b_cv[128]{
      '\x00', '\x01', '\x02', '\x03', '\x04', '\x05', '\x06', '\a',   '\b',
      '\t',   '\n',   '\v',   '\f',   '\r',   '\x0e', '\x0f', '\x10', '\x11',
      '\x12', '\x13', '\x14', '\x15', '\x16', '\x17', '\x18', '\x19', '\x1a',
      '\x1b', '\x1c', '\x1d', '\x1e', '\x1f', ' ',    '!',    '\"',   '#',
      '$',    '%',    '&',    '\'',   '(',    ')',    '*',    '+',    ',',
      '-',    '.',    '/',    '0',    '1',    '2',    '3',    '4',    '5',
      '6',    '7',    '8',    '9',    ':',    ';',    '<',    '=',    '>',
      '?',    '@',    'a',    'b',    'c',    'd',    'e',    'f',    'g',
      'h',    'i',    'j',    'k',    'l',    'm',    'n',    'o',    'p',
      'q',    'r',    's',    't',    'u',    'v',    'w',    'x',    'y',
      'z',    '[',    '\\',   ']',    '^',    '_',    '`',    'a',    'b',
      'c',    'd',    'e',    'f',    'g',    'h',    'i',    'j',    'k',
      'l',    'm',    'n',    'o',    'p',    'q',    'r',    's',    't',
      'u',    'v',    'w',    'x',    'y',    'z',    '{',    '|',    '}',
      '~',    '\x7f'};
  static const char b_cv1[8]{'r', 'e', 'v', 'o', 'l', 'u', 't', 'e'};
  static const char b_cv3[8]{'f', 'l', 'o', 'a', 't', 'i', 'n', 'g'};
  static const char c_vstr[8]{'f', 'l', 'o', 'a', 't', 'i', 'n', 'g'};
  static const char vstr[8]{'r', 'e', 'v', 'o', 'l', 'u', 't', 'e'};
  static const char b_cv2[5]{'f', 'i', 'x', 'e', 'd'};
  static const char b_vstr[5]{'f', 'i', 'x', 'e', 'd'};
  int exitg1;
  int nmatched;
  int partial_match_size_idx_1;
  int u0;
  int u1;
  char partial_match_data[9];
  bool b_bool;
  bool guard1;
  bool guard2;
  bool guard3;
  bool guard4;
  bool matched;
  partial_match_size_idx_1 = 8;
  for (u0 = 0; u0 < 8; u0++) {
    partial_match_data[u0] = ' ';
  }
  nmatched = 0;
  matched = false;
  guard1 = false;
  guard2 = false;
  guard3 = false;
  guard4 = false;
  if (str_size[1] <= 8) {
    b_bool = false;
    u1 = str_size[1];
    u0 = 0;
    do {
      exitg1 = 0;
      if (u0 <= u1 - 1) {
        if (b_cv[static_cast<int>(static_cast<unsigned char>(str_data[u0]) &
                                  127U)] != b_cv[static_cast<int>(b_cv1[u0])]) {
          exitg1 = 1;
        } else {
          u0++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
    if (b_bool) {
      if (str_size[1] == 8) {
        nmatched = 1;
        for (u0 = 0; u0 < 8; u0++) {
          partial_match_data[u0] = vstr[u0];
        }
      } else {
        for (u0 = 0; u0 < 8; u0++) {
          partial_match_data[u0] = vstr[u0];
        }
        matched = true;
        nmatched = 1;
        guard4 = true;
      }
    } else {
      guard4 = true;
    }
  } else {
    guard4 = true;
  }
  if (guard4) {
    if (str_size[1] <= 9) {
      b_bool = false;
      u1 = str_size[1];
      u0 = 0;
      do {
        exitg1 = 0;
        if (u0 <= u1 - 1) {
          if (b_cv[static_cast<int>(static_cast<unsigned char>(str_data[u0]) &
                                    127U)] != b_cv[static_cast<int>(cv3[u0])]) {
            exitg1 = 1;
          } else {
            u0++;
          }
        } else {
          b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
      if (b_bool) {
        if (str_size[1] == 9) {
          nmatched = 1;
          partial_match_size_idx_1 = 9;
          for (u0 = 0; u0 < 9; u0++) {
            partial_match_data[u0] = cv3[u0];
          }
        } else {
          if (!matched) {
            partial_match_size_idx_1 = 9;
            for (u0 = 0; u0 < 9; u0++) {
              partial_match_data[u0] = cv3[u0];
            }
          }
          matched = true;
          nmatched++;
          guard3 = true;
        }
      } else {
        guard3 = true;
      }
    } else {
      guard3 = true;
    }
  }
  if (guard3) {
    if (str_size[1] <= 5) {
      b_bool = false;
      u1 = str_size[1];
      u0 = 0;
      do {
        exitg1 = 0;
        if (u0 <= u1 - 1) {
          if (b_cv[static_cast<int>(static_cast<unsigned char>(str_data[u0]) &
                                    127U)] !=
              b_cv[static_cast<int>(b_cv2[u0])]) {
            exitg1 = 1;
          } else {
            u0++;
          }
        } else {
          b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
      if (b_bool) {
        if (str_size[1] == 5) {
          nmatched = 1;
          partial_match_size_idx_1 = 5;
          for (u0 = 0; u0 < 5; u0++) {
            partial_match_data[u0] = b_vstr[u0];
          }
        } else {
          if (!matched) {
            partial_match_size_idx_1 = 5;
            for (u0 = 0; u0 < 5; u0++) {
              partial_match_data[u0] = b_vstr[u0];
            }
          }
          matched = true;
          nmatched++;
          guard2 = true;
        }
      } else {
        guard2 = true;
      }
    } else {
      guard2 = true;
    }
  }
  if (guard2) {
    if (str_size[1] <= 8) {
      b_bool = false;
      u1 = str_size[1];
      u0 = 0;
      do {
        exitg1 = 0;
        if (u0 <= u1 - 1) {
          if (b_cv[static_cast<int>(static_cast<unsigned char>(str_data[u0]) &
                                    127U)] !=
              b_cv[static_cast<int>(b_cv3[u0])]) {
            exitg1 = 1;
          } else {
            u0++;
          }
        } else {
          b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
      if (b_bool) {
        if (str_size[1] == 8) {
          nmatched = 1;
          partial_match_size_idx_1 = 8;
          for (u0 = 0; u0 < 8; u0++) {
            partial_match_data[u0] = c_vstr[u0];
          }
        } else {
          if (!matched) {
            partial_match_size_idx_1 = 8;
            for (u0 = 0; u0 < 8; u0++) {
              partial_match_data[u0] = c_vstr[u0];
            }
          }
          nmatched++;
          guard1 = true;
        }
      } else {
        guard1 = true;
      }
    } else {
      guard1 = true;
    }
  }
  if (guard1 && (nmatched == 0)) {
    partial_match_size_idx_1 = 8;
    for (u0 = 0; u0 < 8; u0++) {
      partial_match_data[u0] = ' ';
    }
  }
  if ((nmatched == 0) || (str_size[1] == 0)) {
    out_size[0] = 1;
    out_size[1] = 8;
    for (u0 = 0; u0 < 8; u0++) {
      out_data[u0] = ' ';
    }
  } else if (nmatched <= 1) {
    out_size[0] = 1;
    out_size[1] = partial_match_size_idx_1;
    ::std::copy(&partial_match_data[0],
                &partial_match_data[partial_match_size_idx_1], &out_data[0]);
  }
}

} // namespace coder
} // namespace gik9dof

//
// File trailer for validatestring.cpp
//
// [EOF]
//
