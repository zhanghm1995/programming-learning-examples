#include <exception>
#include <cstdio>
#include <cstdarg>
#include <sstream>

namespace noah {

using String = std::string;


/** @see cv::Error::Code */
enum Error {
 CV_StsOk=                       0,  /**< everything is ok                */
 CV_StsBackTrace=               -1,  /**< pseudo error for back trace     */
 CV_StsError=                   -2,  /**< unknown /unspecified error      */
 CV_StsInternal=                -3,  /**< internal error (bad state)      */
 CV_StsNoMem=                   -4,  /**< insufficient memory             */
 CV_StsBadArg=                  -5,  /**< function arg/param is bad       */
 CV_StsBadFunc=                 -6,  /**< unsupported function            */
 CV_StsNoConv=                  -7,  /**< iter. didn't converge           */
 CV_StsAutoTrace=               -8,  /**< tracing                         */
 CV_HeaderIsNull=               -9,  /**< image header is NULL            */
 CV_BadImageSize=              -10,  /**< image size is invalid           */
 CV_BadOffset=                 -11,  /**< offset is invalid               */
 CV_BadDataPtr=                -12,  /**/
 CV_BadStep=                   -13,  /**< image step is wrong, this may happen for a non-continuous matrix */
 CV_BadModelOrChSeq=           -14,  /**/
 CV_BadNumChannels=            -15,  /**< bad number of channels, for example, some functions accept only single channel matrices */
 CV_BadNumChannel1U=           -16,  /**/
 CV_BadDepth=                  -17,  /**< input image depth is not supported by the function */
 CV_BadAlphaChannel=           -18,  /**/
 CV_BadOrder=                  -19,  /**< number of dimensions is out of range */
 CV_BadOrigin=                 -20,  /**< incorrect input origin               */
 CV_BadAlign=                  -21,  /**< incorrect input align                */
 CV_BadCallBack=               -22,  /**/
 CV_BadTileSize=               -23,  /**/
 CV_BadCOI=                    -24,  /**< input COI is not supported           */
 CV_BadROISize=                -25,  /**< incorrect input roi                  */
 CV_MaskIsTiled=               -26,  /**/
 CV_StsNullPtr=                -27,  /**< null pointer */
 CV_StsVecLengthErr=           -28,  /**< incorrect vector length */
 CV_StsFilterStructContentErr= -29,  /**< incorrect filter structure content */
 CV_StsKernelStructContentErr= -30,  /**< incorrect transform kernel content */
 CV_StsFilterOffsetErr=        -31,  /**< incorrect filter offset value */
 CV_StsBadSize=                -201, /**< the input/output structure size is incorrect  */
 CV_StsDivByZero=              -202, /**< division by zero */
 CV_StsInplaceNotSupported=    -203, /**< in-place operation is not supported */
 CV_StsObjectNotFound=         -204, /**< request can't be completed */
 CV_StsUnmatchedFormats=       -205, /**< formats of input/output arrays differ */
 CV_StsBadFlag=                -206, /**< flag is wrong or not supported */
 CV_StsBadPoint=               -207, /**< bad CvPoint */
 CV_StsBadMask=                -208, /**< bad format of mask (neither 8uC1 nor 8sC1)*/
 CV_StsUnmatchedSizes=         -209, /**< sizes of input/output structures do not match */
 CV_StsUnsupportedFormat=      -210, /**< the data format/type is not supported by the function*/
 CV_StsOutOfRange=             -211, /**< some of parameters are out of range */
 CV_StsParseError=             -212, /**< invalid syntax/structure of the parsed file */
 CV_StsNotImplemented=         -213, /**< the requested function/feature is not implemented */
 CV_StsBadMemBlock=            -214, /**< an allocated block has been corrupted */
 CV_StsAssert=                 -215, /**< assertion failed   */
 CV_GpuNotSupported=           -216, /**< no CUDA support    */
 CV_GpuApiCallError=           -217, /**< GPU API call error */
 CV_OpenGlNotSupported=        -218, /**< no OpenGL support  */
 CV_OpenGlApiCallError=        -219, /**< OpenGL API call error */
 CV_OpenCLApiCallError=        -220, /**< OpenCL API call error */
 CV_OpenCLDoubleNotSupported=  -221,
 CV_OpenCLInitError=           -222, /**< OpenCL initialization error */
 CV_OpenCLNoAMDBlasFft=        -223
};

const char* cvErrorStr( int status )
{
    static char buf[256];

    switch (status)
    {
    case CV_StsOk :                  return "No Error";
    case CV_StsBackTrace :           return "Backtrace";
    case CV_StsError :               return "Unspecified error";
    case CV_StsInternal :            return "Internal error";
    case CV_StsNoMem :               return "Insufficient memory";
    case CV_StsBadArg :              return "Bad argument";
    case CV_StsNoConv :              return "Iterations do not converge";
    case CV_StsAutoTrace :           return "Autotrace call";
    case CV_StsBadSize :             return "Incorrect size of input array";
    case CV_StsNullPtr :             return "Null pointer";
    case CV_StsDivByZero :           return "Division by zero occurred";
    case CV_BadStep :                return "Image step is wrong";
    case CV_StsInplaceNotSupported : return "Inplace operation is not supported";
    case CV_StsObjectNotFound :      return "Requested object was not found";
    case CV_BadDepth :               return "Input image depth is not supported by function";
    case CV_StsUnmatchedFormats :    return "Formats of input arguments do not match";
    case CV_StsUnmatchedSizes :      return "Sizes of input arguments do not match";
    case CV_StsOutOfRange :          return "One of arguments\' values is out of range";
    case CV_StsUnsupportedFormat :   return "Unsupported format or combination of formats";
    case CV_BadCOI :                 return "Input COI is not supported";
    case CV_BadNumChannels :         return "Bad number of channels";
    case CV_StsBadFlag :             return "Bad flag (parameter or structure field)";
    case CV_StsBadPoint :            return "Bad parameter of type CvPoint";
    case CV_StsBadMask :             return "Bad type of mask argument";
    case CV_StsParseError :          return "Parsing error";
    case CV_StsNotImplemented :      return "The function/feature is not implemented";
    case CV_StsBadMemBlock :         return "Memory block has been corrupted";
    case CV_StsAssert :              return "Assertion failed";
    case CV_GpuNotSupported :        return "No CUDA support";
    case CV_GpuApiCallError :        return "Gpu API call";
    case CV_OpenGlNotSupported :     return "No OpenGL support";
    case CV_OpenGlApiCallError :     return "OpenGL API call";
    };

    sprintf(buf, "Unknown %s code %d", status >= 0 ? "status":"error", status);
    return buf;
}

class Exception : public std::exception
{
public:
    /*!
     Default constructor
     */
    Exception();
    /*!
     Full constructor. Normally the constructor is not called explicitly.
     Instead, the macros CV_Error(), CV_Error_() and CV_Assert() are used.
    */
    Exception(int _code, const String& _err, const String& _func, const String& _file, int _line);
    virtual ~Exception() throw();

    /*!
     \return the error description and the context as a text string.
    */
    virtual const char *what() const throw() override;
    void formatMessage();

    String msg; ///< the formatted error message

    int code; ///< error code @see CVStatus
    String err; ///< error description
    String func; ///< function name. Available only when the compiler supports getting it
    String file; ///< source file name where the error has occurred
    int line; ///< line number in the source file where the error has occurred
};

Exception::Exception() { code = 0; line = 0; }

Exception::Exception(int _code, const String& _err, const String& _func, const String& _file, int _line)
: code(_code), err(_err), func(_func), file(_file), line(_line)
{
    formatMessage();
}

Exception::~Exception() throw() {}

/*!
 \return the error description and the context as a text string.
 */
const char* Exception::what() const throw() { return msg.c_str(); }

void Exception::formatMessage()
{
    msg = "zhanghaiming\n";
    // if (func.size() > 0)
    // {
    //     msg = sprintf("OpenCV(%s) %s:%d: error: (%d:%s) %s in function '%s'\n", "3.4.4", file.c_str(), line, code, cvErrorStr(code), err.c_str(), func.c_str());
    // }
    // else
    // {
    //     msg = sprintf("OpenCV(%s) %s:%d: error: (%d:%s) %s%s", "3.4.4", file.c_str(), line, code, cvErrorStr(code), err.c_str(), false ? "" : "\n");
    // }
}

int cv_vsnprintf(char* buf, int len, const char* fmt, va_list args)
{
    return vsnprintf(buf, len, fmt, args);
}

int cv_snprintf(char* buf, int len, const char* fmt, ...)
{
    va_list va;
    va_start(va, fmt);
    int res = cv_vsnprintf(buf, len, fmt, va);
    va_end(va);
    return res;
}

static void dumpException(const Exception& exc)
{
    const char* errorStr = cvErrorStr(exc.code);
    char buf[1 << 12];

    cv_snprintf(buf, sizeof(buf),
        "OpenCV(%s) Error: %s (%s) in %s, file %s, line %d",
        "CV_VERSION",
        errorStr, exc.err.c_str(), exc.func.size() > 0 ?
        exc.func.c_str() : "unknown function", exc.file.c_str(), exc.line);

    fflush(stdout); fflush(stderr);
    fprintf(stderr, "%s\n", buf);
    fflush(stderr);
}

void error( const Exception& exc )
{
    dumpException(exc);

    throw exc;
}

void error(int _code, const String& _err, const char* _func, const char* _file, int _line)
{
    error(noah::Exception(_code, _err, _func, _file, _line));
}

}

#define Noah_Assert( expr ) do { if(!!(expr)) ; else noah::error( noah::Error::CV_StsAssert, #expr, __func__, __FILE__, __LINE__ ); } while(0)