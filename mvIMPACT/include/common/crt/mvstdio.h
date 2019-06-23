//-----------------------------------------------------------------------------
#ifndef mvstdioH
#define mvstdioH mvstdioH
//-----------------------------------------------------------------------------
#include <errno.h>
#include <stdio.h>
#if defined(_MSC_VER)
#   include <share.h>
#endif

//-----------------------------------------------------------------------------
/// \brief Version that mimics the C11 \c fopen_s function.
/**
 * See \c fopen_s of your runtime implementation for documentation!
 */
inline int mv_fopen_s( FILE** ppFile, const char* pName, const char* pMode )
//-----------------------------------------------------------------------------
{
#if defined(_MSC_VER) && (_MSC_VER >= 1400) // is at least VC 2005 compiler?
    // The M$ version of fopen_s does not support sharing thus parallel access to the same file...
    // As this is needed by mvIMPACT Acquire e.g. when writing log files from multiple processes into
    // a common file we have to use the '_fsopen' function with '_SH_DENYNO' here to get
    // what we want, which is parallel access AND no compiler warnings
    *ppFile = _fsopen( pName, pMode, _SH_DENYNO );
    return errno;
#elif defined (__STDC_LIB_EXT1__)
    return fopen_s( ppFile, pName, pMode );
#else
    *ppFile = fopen( pName, pMode );
    return errno;
#endif // #if (defined(_MSC_VER) && (_MSC_VER >= 1400)) || defined (__STDC_LIB_EXT1__) // is at least VC 2005 compiler OR implementation supports CRT extensions?
}

//-----------------------------------------------------------------------------
/// \brief Version that mimics the traditional \c fopen function.
/**
 * See \c fopen of your runtime implementation for documentation!
 */
inline FILE* mv_fopen_s( const char* pName, const char* pMode )
//-----------------------------------------------------------------------------
{
    FILE* pFile = 0;
    mv_fopen_s( &pFile, pName, pMode );
    return pFile;
}

#endif // mvstdioH
