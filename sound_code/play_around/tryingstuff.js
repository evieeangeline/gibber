
// Home

Clock.bpm = 70
Theory.root = 'c#5'
Theory.mode = 'dorian'

Theory.degree.seq( ['i','-iv', 'ii', '-V', 'iii'], [4,2,1] )

// Global Effects
verb  = Reverb( 'space' ).bus() 
delay = Delay( '1/6' ).bus()
 
long_delay = Delay( '1/4' ).bus()

// bass
bass = Synth('bass.hollow', { saturation:20, gain:.8 })
  .connect( long_delay, 1 )
  .connect( verb, 1 )
 
bass.note.tidal( '0' )


// Drums
drums = Drums()
drums.loudness = 1
drums.fx.add( Distortion({ pregain:1.5, postgain:1 }) )
 
drums.tidal('kd kd kd kd')
 

// Hats

hat = Hat({ gain:.075 })
hat.trigger.seq( [1,.5], [1/4, 1/16,1/16, 1/16, 1/16] )
hat.decay = gen( .02 + cycle( beats(16) * 4 ) * .0125 )
hat.fx.add( Distortion({ pregain:100, postgain:.1 }) )
 
pad = Synth[4]('rhodes', { decay:8, gain:.15 })
pad.fx[0].connect( Out, .125)
pad.fx[0].connect( verb, 1 )
pad.chord.seq([[0,2,4,6], [1,2,4,7]], 4 )
 

// clarient

clar = FM('clarinet', { saturation:20, gain:.1})
clar.feedback = 0
gen( cycle( beats))
clar.cutoff = gen( .02 + cycle( beats(16) * 4 ) * .0125 )

clar.note.tidal('0 2 4 6 8 10 0 2 4 6 8 10 0 2 4 6 8 10')