// Copyright (C) 2008-2014 LAAS-CNRS, JRL AIST-CNRS.
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#ifndef HQP_WARNING_HH
# define HQP_WARNING_HH

// Emits a warning in a portable way.
//
// To emit a warning, one can insert:
//
// #pragma message HQP_WARN("your warning message here")
//
// The use of this syntax is required as this is /not/ a standardized
// feature of C++ language or preprocessor, even if most of the
// compilers support it.

# define HQP_WARN_STRINGISE_IMPL(x) #x
# define HQP_WARN_STRINGISE(x) \
         HQP_WARN_STRINGISE_IMPL(x)
# ifdef __GNUC__
#   define HQP_WARN(exp) ("WARNING: " exp)
# else
#  ifdef _MSC_VER
#   define FILE_LINE_LINK __FILE__ "(" \
           HQP_WARN_STRINGISE(__LINE__) ") : "
#   define HQP_WARN(exp) (FILE_LINE_LINK "WARNING: " exp)
#  else
// If the compiler is not recognized, drop the feature.
#   define HQP_WARN(MSG) /* nothing */
#  endif // __MSVC__
# endif // __GNUC__

#endif //! HQP_WARNING_HH
