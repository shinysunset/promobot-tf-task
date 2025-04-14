#ifndef PROMOBOT_TF_LIBRARY__VISIBILITY_CONTROL_H_
#define PROMOBOT_TF_LIBRARY__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define PROMOBOT_TF_LIBRARY_EXPORT __attribute__ ((dllexport))
    #define PROMOBOT_TF_LIBRARY_IMPORT __attribute__ ((dllimport))
  #else
    #define PROMOBOT_TF_LIBRARY_EXPORT __declspec(dllexport)
    #define PROMOBOT_TF_LIBRARY_IMPORT __declspec(dllimport)
  #endif
  #ifdef PROMOBOT_TF_LIBRARY_BUILDING_LIBRARY
    #define PROMOBOT_TF_LIBRARY_PUBLIC PROMOBOT_TF_LIBRARY_EXPORT
  #else
    #define PROMOBOT_TF_LIBRARY_PUBLIC PROMOBOT_TF_LIBRARY_IMPORT
  #endif
  #define PROMOBOT_TF_LIBRARY_PUBLIC_TYPE PROMOBOT_TF_LIBRARY_PUBLIC
  #define PROMOBOT_TF_LIBRARY_LOCAL
#else
  #define PROMOBOT_TF_LIBRARY_EXPORT __attribute__ ((visibility("default")))
  #define PROMOBOT_TF_LIBRARY_IMPORT
  #if __GNUC__ >= 4
    #define PROMOBOT_TF_LIBRARY_PUBLIC __attribute__ ((visibility("default")))
    #define PROMOBOT_TF_LIBRARY_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define PROMOBOT_TF_LIBRARY_PUBLIC
    #define PROMOBOT_TF_LIBRARY_LOCAL
  #endif
  #define PROMOBOT_TF_LIBRARY_PUBLIC_TYPE
#endif

#endif  // PROMOBOT_TF_LIBRARY__VISIBILITY_CONTROL_H_
