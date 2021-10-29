
#ifndef ARIS_API_H
#define ARIS_API_H

#ifdef ARIS_LIB_STATIC_DEFINE
#  define ARIS_API
#  define ARIS_LIB_NO_EXPORT
#else
#  ifndef ARIS_API
#    ifdef aris_lib_EXPORTS
        /* We are building this library */
#      define ARIS_API __attribute__((visibility("default")))
#    else
        /* We are using this library */
#      define ARIS_API __attribute__((visibility("default")))
#    endif
#  endif

#  ifndef ARIS_LIB_NO_EXPORT
#    define ARIS_LIB_NO_EXPORT __attribute__((visibility("hidden")))
#  endif
#endif

#ifndef ARIS_LIB_DEPRECATED
#  define ARIS_LIB_DEPRECATED __attribute__ ((__deprecated__))
#endif

#ifndef ARIS_LIB_DEPRECATED_EXPORT
#  define ARIS_LIB_DEPRECATED_EXPORT ARIS_API ARIS_LIB_DEPRECATED
#endif

#ifndef ARIS_LIB_DEPRECATED_NO_EXPORT
#  define ARIS_LIB_DEPRECATED_NO_EXPORT ARIS_LIB_NO_EXPORT ARIS_LIB_DEPRECATED
#endif

#if 0 /* DEFINE_NO_DEPRECATED */
#  ifndef ARIS_LIB_NO_DEPRECATED
#    define ARIS_LIB_NO_DEPRECATED
#  endif
#endif

#endif /* ARIS_API_H */
