// Copyright 2018 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef MESSAGE_FILTERS__VISIBILITY_CONTROL_H_
#define MESSAGE_FILTERS__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define MESSAGE_FILTERS_EXPORT __attribute__ ((dllexport))
    #define MESSAGE_FILTERS_IMPORT __attribute__ ((dllimport))
  #else
    #define MESSAGE_FILTERS_EXPORT __declspec(dllexport)
    #define MESSAGE_FILTERS_IMPORT __declspec(dllimport)
  #endif
  #ifdef MESSAGE_FILTERS_BUILDING_DLL
    #define MESSAGE_FILTERS_PUBLIC MESSAGE_FILTERS_EXPORT
  #else
    #define MESSAGE_FILTERS_PUBLIC MESSAGE_FILTERS_IMPORT
  #endif
  #define MESSAGE_FILTERS_LOCAL
#else
  #define MESSAGE_FILTERS_EXPORT __attribute__ ((visibility("default")))
  #define MESSAGE_FILTERS_IMPORT
  #if __GNUC__ >= 4
    #define MESSAGE_FILTERS_PUBLIC __attribute__ ((visibility("default")))
    #define MESSAGE_FILTERS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define MESSAGE_FILTERS_PUBLIC
    #define MESSAGE_FILTERS_LOCAL
  #endif
#endif

#ifdef __cplusplus
}
#endif

#endif  // MESSAGE_FILTERS__VISIBILITY_CONTROL_H_

