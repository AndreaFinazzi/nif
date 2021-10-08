
#ifndef LUMINLIB_EXPORT_H
#define LUMINLIB_EXPORT_H

#ifdef LUMINLIB_STATIC_DEFINE
#  define LUMINLIB_EXPORT
#  define LUMINLIB_NO_EXPORT
#else
#  ifndef LUMINLIB_EXPORT
#    ifdef LuminLib_EXPORTS
        /* We are building this library */
#      define LUMINLIB_EXPORT 
#    else
        /* We are using this library */
#      define LUMINLIB_EXPORT 
#    endif
#  endif

#  ifndef LUMINLIB_NO_EXPORT
#    define LUMINLIB_NO_EXPORT 
#  endif
#endif

#ifndef LUMINLIB_DEPRECATED
#  define LUMINLIB_DEPRECATED __attribute__ ((__deprecated__))
#endif

#ifndef LUMINLIB_DEPRECATED_EXPORT
#  define LUMINLIB_DEPRECATED_EXPORT LUMINLIB_EXPORT LUMINLIB_DEPRECATED
#endif

#ifndef LUMINLIB_DEPRECATED_NO_EXPORT
#  define LUMINLIB_DEPRECATED_NO_EXPORT LUMINLIB_NO_EXPORT LUMINLIB_DEPRECATED
#endif

#if 0 /* DEFINE_NO_DEPRECATED */
#  ifndef LUMINLIB_NO_DEPRECATED
#    define LUMINLIB_NO_DEPRECATED
#  endif
#endif

#endif /* LUMINLIB_EXPORT_H */
