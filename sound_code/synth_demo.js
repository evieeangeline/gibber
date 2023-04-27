Clock.bpm = 140
 
verb = Reverb( 'space' ).bus() 
 

// Lead A - creepy distortedey high pitched synth, triggered off sine wave
leadA = Synth( 'cry', { gain:.1, octave:1 })
  .connect( verb, 1 )
  .note.seq( sine( .15, 7 ) , [1/2,1,2] )

leadA.stop()
