#ifndef GCS_EXPORT_HPP
#define GCS_EXPORT_HPP

#if defined(_WIN32) || defined(__CYGWIN__)
#if defined(GCS_BUILDING_LIBRARY)
#define GCS_API __declspec(dllexport)
#else
#define GCS_API __declspec(dllimport)
#endif
#define GCS_NO_EXPORT
#else
#if __GNUC__ >= 4
#define GCS_API __attribute__((visibility("default")))
#define GCS_NO_EXPORT __attribute__((visibility("hidden")))
#else
#define GCS_API
#define GCS_NO_EXPORT
#endif
#endif

#endif // GCS_EXPORT_HPP
