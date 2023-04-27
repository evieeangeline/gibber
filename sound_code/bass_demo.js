
Clock.bpm = 120
Theory.root = 'd#4'
Theory.mode = 'dorian'
 
verb  = Reverb( 'space' ).bus() 
delay = Delay( '1/6' ).bus()

// Bass A - acidey synthey bass, with moving Q and cuttoff. 
// (Q determines sharpness) 

bassA = Synth('acidBass2', { saturation:20, gain:.3 })
  .connect( delay, .25 )
  .connect( verb, .125 )
 
bassA.note.tidal( '0 0 0 0 4 6 0 ~ 0 ~ 7 -7 ~ 0 -7 0' )
bassA.decay.seq( [1/32, 1/16], 1/2 )
bassA.glide.seq( [1,1,100,100 ], 1/4 )
bassA.Q = gen( 0.5 + cycle(0.1) * 0.49 )
bassA.cutoff = gen( 0.5 + cycle(0.07) * 0.45 )  
    
bassA.stop()


Theory.degree.seq( ['i','-iv','-V'], [8,4,4] )


// Bass B - dark and stormy bass

bassB = Monosynth( 'bassPad', { decay:4 })
  .connect( verb, .5 )
  .note.seq( [0, 0, -1, -1, -2, -2, -4, -4], [0.25, 3.75] )
 
bassB.stop()

