Clock.bpm = 120
Theory.root = 'f#4'
Theory.mode = 'phrygian'
 
verb  = Reverb( 'space' ).bus() 
delay = Delay( '1/6' ).bus()

Theory.degree.seq( ['i','-iv','-V'], [8,4,4] )

// Bass A - acidey synthey bass, with moving Q and cuttoff. 
// (Q determines sharpness) 

Theory.root = 'd#4' // OR use d#5 for a mid synth
acidBass = Synth('acidBass2', { saturation:20, gain:.3 })
  .connect( delay, .25 )
  .connect( verb, .125 )
 
acidBass.note.tidal( '0 0 0 0 4 6 0 ~ 0 ~ 7 -7 ~ 0 -7 0' )
acidBass.decay.seq( [1/32, 1/16], 1/2 )
acidBass.glide.seq( [1,1,100,100 ], 1/4 )
acidBass.Q = gen( 0.5 + cycle(0.1) * 0.49 )
acidBass.cutoff = gen( 0.5 + cycle(0.07) * 0.45 )  
    
acidBass.stop()


// Bass B - dark and stormy bass

Theory.root = 'd#5' 
darkBass = Monosynth( 'bassPad', { decay:4 })
  .connect( verb, .5 )
  .note.seq( [0, 0, -1, -1, -2, -2, -4, -4], [0.25, 3.75] )
 
darkBass.stop()


// Bass C - FM Synth 

fmBass = FM('bass')

fmBass.note.seq( [0,-7], [1/8])

fmBass.decay = gen( 0.5 + cycle(0.1) *1)
fmBass.attack = gen( 0.4 + cycle(0.06) *0)
fmBass.cutoff = 0.2

fmBass.stop()