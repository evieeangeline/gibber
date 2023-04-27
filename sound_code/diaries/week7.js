//settings
Clock.bpm = 170
Theory.root = 'c#5' 
Theory.mode = 'dorian' 

verb = Reverb( 'space' ).bus()
delay = Delay( '1/8').bus()

Theory.degree.seq( ['i', '-iv', '-v'], [8, 4, 4])

//pad 
pad = Synth[4]('rhodes', {decay:8, gain:.15})
pad.fx[0].connect( verb, 1) 
pad.chord.seq([[0,2,4,6]], 4) 


//drums 
drumsA = Drums()
drumsA.fx.add( Distortion({ pregain:1.5, postgain:1 }) )
 
drumsA.tidal('kd [kd, sd]  kd [kd sd] ')

//hat
hat = Hat({ gain:.075 })
hat.trigger.seq( [1,.5], [1/8, 1/16] )
hat.decay = gen( .02 + cycle( beats(16) * 4 ) * .0125 )
hat.fx.add( Distortion({ pregain:100, postgain:.1 }) )


//bass
bassA = Synth('acidBass2', { saturation:20, gain:.3 })
  .connect( delay, .25 )
  .connect( verb, .125 )


bassA.cutoff = 0.2
bassA.cutoff = gen( 0.4 + cycle(0.06) *0.2)

bassA.note.tidal( '0 ~ ~ 0 ~ ~ 0 ~ -2 ~ ~ -2 ~ ~ 3 -3' )