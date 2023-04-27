  
Clock.bpm = 120
Theory.root = 'd#4'
Theory.mode = 'dorian'
 
verb  = Reverb( 'space' ).bus() 
delay = Delay( '1/6' ).bus()
 

// PadA : rhodes keys with lots of wah wah 

padA = Synth[4]('rhodes', { decay:8, gain:.15 })
padA.fx[0].connect( Out, .125)
padA.fx[0].connect( verb, 1 )
padA.chord.seq([[0,2,4,6], [1,2,4,7]], 4 )

padA.stop()
 
Theory.degree.seq( ['i','-iv','-V'], [8,4,4] )


// Synth A: 