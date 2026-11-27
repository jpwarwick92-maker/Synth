// The drawn wave: sixteen points, joined by straight lines, that the DRAW
// stop of the Wave selector plays.
//
// One drawing belongs to each patch and is shared by both oscillators.
// Under the plain engine it is the oscillator's wave; under the wavetable
// engine it is table 6, morphing towards a sine; under the pluck engine it
// is the shape the string is struck with. (Under FM the sixth stop is a
// ratio, not the drawing.)
//
// A drawing can't be played as it stands: its corners would fold back as
// false tones on high notes. So it is turned into the same kind of tables
// the saw has, one for each octave, each holding only the harmonics that
// fit below half the sample rate there. That takes far longer than an
// audio block, so it is done in the spare time between other work (work()),
// a slice at a time, into a second copy, while the audio goes on playing
// the first. When the new copy is ready the audio crossfades to it over
// 20 ms, so a drawing can be changed while a note sounds without a click.
//
// Who may touch what: the control context edits the points and builds the
// idle copy; the audio context reads the live copy and does the swap. The
// idle copy is written only once the audio has taken the last one into use
// and finished fading to it.
#pragma once

#include "fw_config.h"
#include "tables.h"

namespace fw {

constexpr int kDrawPoints = 16;
constexpr int kDrawLevels = 8;  // 128, 64, ... 1 harmonics
constexpr int kDrawHarmonics = 128;
constexpr int kDrawGainPoints = 9;  // softening loudness at Macro 0, 1/8, ... 1

// Starting shapes on the DRAW page.
enum DrawShape : uint8_t { DRAWSHAPE_SAW, DRAWSHAPE_SQUARE, DRAWSHAPE_TRI, DRAWSHAPE_SINE, DRAWSHAPE_FLAT, DRAWSHAPE_COUNT };
extern const char* const kDrawShapeNames[DRAWSHAPE_COUNT];

// One prepared drawing.
struct DrawCopy {
  int16_t tab[kDrawLevels][kTabLen + 1];  // the plain engine's tables, level 0 the richest
  int16_t wt[4][6][512 + 1];              // the wavetable engine's frames (as gWt: frames, levels, samples)
  int8_t pts[kDrawPoints];                // the points it was built from
  float gain;                             // a table reading times this is the drawing at a saw's loudness
  float softGain[kDrawGainPoints];        // and this keeps that loudness as Macro softens it
  float pluckMean, pluckGain;             // the exciter: the points less their mean, times this
  bool silent;                            // the drawing is flat: it makes no sound
};

class Drawing {
 public:
  // The INIT drawing (a saw), not yet built.
  void reset();
  // Builds the current points at once. Start-up only, before the audio runs.
  void buildNow();

  // ---- control context ----
  int8_t point(int i) const { return pts_[i & (kDrawPoints - 1)]; }
  const int8_t* points() const { return pts_; }
  // Each of these returns true if the drawing changed.
  bool setPoint(int i, int v);
  bool setPoints(const int8_t* p);
  bool startShape(int shape);
  bool smooth();
  // One slice of the work of preparing the drawing. Returns true while there is more to do.
  bool work();
  bool settled() const;  // what is heard is the drawing as it stands
  bool silent() const;   // the drawing as it stands is flat

  // ---- audio context ----
  // At the start of each control block: takes a finished copy into use,
  // and moves the crossfade on by `len` samples.
  void audioBlock(int len, float sampleRate);
  const DrawCopy& live() const;
  const DrawCopy& old() const;  // the copy being faded from
  float fade() const { return fadeNow_; }  // 1 = all old copy, 0 = all live; at the start of this block
  float fadeStep() const { return fadeStep_; }  // how much it falls each sample

 private:
  void beginBuild();
  void step();

  int8_t pts_[kDrawPoints];
  uint32_t edits_;      // goes up with every change to the points
  uint32_t built_;      // the edits_ value the live copy (or the one being built) holds

  // build in progress (control context)
  bool building_;
  int target_;          // the copy being written
  int stage_, item_;
  float re_[kDrawPoints], im_[kDrawPoints];  // the polyline's spectrum, which repeats every 16 harmonics
  float peak_;

  // handover (each written by one side only)
  volatile int8_t live_;      // audio: the copy being played
  volatile int8_t oldCopy_;   // audio
  volatile bool ready_;       // control sets it when a copy is built; audio clears it when it takes it
  volatile int8_t readyCopy_;
  volatile float fade_;       // audio: how far through the crossfade, 1 at its start
  float fadeNow_, fadeStep_;
};

extern Drawing gDraw;

// The table level to read for a note advancing `inc` cycles per sample:
// the richest whose top harmonic is below half the sample rate.
int drawLevelFor(float inc);

}  // namespace fw
