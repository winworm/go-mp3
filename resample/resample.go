package resample

/*
#cgo LDFLAGS: ${SRCDIR}/libsoxr.a -lm

#include <stdlib.h>
#include "soxr.h"
*/
import "C"
import (
	"errors"
	"runtime"
	"unsafe"
)

const (
	// Quality settings
	Quick     = 0 // Quick cubic interpolation
	LowQ      = 1 // LowQ 16-bit with larger rolloff
	MediumQ   = 2 // MediumQ 16-bit with medium rolloff
	HighQ     = 4 // High quality
	VeryHighQ = 6 // Very high quality

	// Input formats
	F32 = 0 // 32-bit floating point PCM
	F64 = 1 // 64-bit floating point PCM
	I32 = 2 // 32-bit signed linear PCM
	I16 = 3 // 16-bit signed linear PCM

	byteLen = 8
)

// Resampler resamples PCM sound data.
type Resampler struct {
	resampler C.soxr_t
	inRate    float64 // input sample rate
	outRate   float64 // output sample rate
	channels  int     // number of input channels
	frameSize int     // frame size in bytes
}

var threads int

func init() {
	threads = runtime.NumCPU()
}

func New(inputRate, outputRate float64, channels, format, quality int) (*Resampler, error) {
	var err error
	var size int
	if inputRate <= 0 || outputRate <= 0 {
		return nil, errors.New("Invalid input or output sampling rates")
	}
	if channels == 0 {
		return nil, errors.New("Invalid channels number")
	}
	if quality < 0 || quality > 6 {
		return nil, errors.New("Invalid quality setting")
	}
	switch format {
	case F64:
		size = 64 / byteLen
	case F32, I32:
		size = 32 / byteLen
	case I16:
		size = 16 / byteLen
	default:
		return nil, errors.New("Invalid format setting")
	}
	var soxr C.soxr_t
	var soxErr C.soxr_error_t
	// Setup soxr and create a stream resampler
	ioSpec := C.soxr_io_spec(C.soxr_datatype_t(format), C.soxr_datatype_t(format))
	qSpec := C.soxr_quality_spec(C.ulong(quality), 0)
	runtimeSpec := C.soxr_runtime_spec(C.uint(threads))
	soxr = C.soxr_create(C.double(inputRate), C.double(outputRate), C.uint(channels), &soxErr, &ioSpec, &qSpec, &runtimeSpec)
	if C.GoString(soxErr) != "" && C.GoString(soxErr) != "0" {
		err = errors.New(C.GoString(soxErr))
		C.free(unsafe.Pointer(soxErr))
		return nil, err
	}

	r := Resampler{
		resampler: soxr,
		inRate:    inputRate,
		outRate:   outputRate,
		channels:  channels,
		frameSize: size,
	}
	C.free(unsafe.Pointer(soxErr))
	return &r, err
}

// Close clean-ups and frees memory. Should always be called when
// finished using the resampler.
func (r *Resampler) Close() (err error) {
	if r.resampler == nil {
		return errors.New("soxr resampler is nil")
	}
	C.soxr_delete(r.resampler)
	r.resampler = nil
	return
}

func (r *Resampler) toChannels(src []byte, len, channels int) []byte {
	buf := make([]byte, len/channels)
	for i, n := 0, 0; i < len; i += channels {
		if i%4 == 0 {
			for j := 0; j < channels; j++ {
				copy(buf[n+j:], src[i+j:])
			}
			n += channels
		}
	}
	//fmt.Println("len:", len)
	return buf
}

// Write resamples PCM sound data. Writes len(p) bytes from p to
// the underlying data stream, returns the number of bytes written
// from p (0 <= n <= len(p)) and any error encountered that caused
// the write to stop early.
func (r *Resampler) Write(p []byte) (out []byte, err error) {
	//fmt.Println("Write channels:", r.channels)
	if r.resampler == nil {
		err = errors.New("soxr resampler is nil")
		return
	}
	if len(p) == 0 {
		return
	}
	framesIn := len(p) / r.frameSize / r.channels
	if framesIn == 0 {
		err = errors.New("Incomplete input frame data")
		return
	}
	if len(p)%(r.frameSize*r.channels) != 0 {
		err = errors.New("Fragmented last frame in input data")
		return
	}
	framesOut := int(float64(framesIn) * (r.outRate / r.inRate))
	if framesOut == 0 {
		err = errors.New("Not enough input to generate output")
		return
	}

	dataIn := C.CBytes(p)
	dataOut := C.malloc(C.size_t(framesOut*r.channels*r.frameSize + 4))
	var soxErr C.soxr_error_t
	var read1, read2, done1, done2 C.size_t = 0, 0, 0, 0
	buf := make([]byte, 0)
	soxErr = C.soxr_process(r.resampler, C.soxr_in_t(dataIn), C.size_t(framesIn), &read1, C.soxr_out_t(dataOut), C.size_t(framesOut), &done1)
	if C.GoString(soxErr) != "" && C.GoString(soxErr) != "0" {
		err = errors.New(C.GoString(soxErr))
		goto cleanup
	}

	buf = append(buf, C.GoBytes(dataOut, C.int(int(done1)*r.channels*r.frameSize))...)
	// Consume any data left in the resampling filter
	soxErr = C.soxr_process(r.resampler, C.soxr_in_t(nil), C.size_t(0), &read2, C.soxr_out_t(dataOut), C.size_t(framesOut), &done2)
	if C.GoString(soxErr) != "" && C.GoString(soxErr) != "0" {
		err = errors.New(C.GoString(soxErr))
		goto cleanup
	}

	buf = append(buf, C.GoBytes(dataOut, C.int(int(done2)*r.channels*r.frameSize))...)
	// If we have read all input and flushed all output, avoid to report short writes due
	// to output frames missing because of downsampling or other odd reasons.
	out = r.toChannels(buf, len(buf), 2)
cleanup:
	C.free(dataIn)
	C.free(dataOut)
	C.free(unsafe.Pointer(soxErr))
	return
}

// Reset permits reusing a Resampler rather than allocating a new one.
func (r *Resampler) Reset() (err error) {
	if r.resampler == nil {
		return errors.New("soxr resampler is nil")
	}
	C.soxr_clear(r.resampler)
	return
}