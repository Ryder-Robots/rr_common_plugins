#ifndef RR_COMMON_PLUGINS__VISIBILITY_CONTROL_H_
#define RR_COMMON_PLUGINS__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define RR_COMMON_PLUGINS_EXPORT __attribute__ ((dllexport))
    #define RR_COMMON_PLUGINS_IMPORT __attribute__ ((dllimport))
  #else
    #define RR_COMMON_PLUGINS_EXPORT __declspec(dllexport)
    #define RR_COMMON_PLUGINS_IMPORT __declspec(dllimport)
  #endif
  #ifdef RR_COMMON_PLUGINS_BUILDING_LIBRARY
    #define RR_COMMON_PLUGINS_PUBLIC RR_COMMON_PLUGINS_EXPORT
  #else
    #define RR_COMMON_PLUGINS_PUBLIC RR_COMMON_PLUGINS_IMPORT
  #endif
  #define RR_COMMON_PLUGINS_PUBLIC_TYPE RR_COMMON_PLUGINS_PUBLIC
  #define RR_COMMON_PLUGINS_LOCAL
#else
  #define RR_COMMON_PLUGINS_EXPORT __attribute__ ((visibility("default")))
  #define RR_COMMON_PLUGINS_IMPORT
  #if __GNUC__ >= 4
    #define RR_COMMON_PLUGINS_PUBLIC __attribute__ ((visibility("default")))
    #define RR_COMMON_PLUGINS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define RR_COMMON_PLUGINS_PUBLIC
    #define RR_COMMON_PLUGINS_LOCAL
  #endif
  #define RR_COMMON_PLUGINS_PUBLIC_TYPE
#endif

#endif  // RR_COMMON_PLUGINS__VISIBILITY_CONTROL_H_
