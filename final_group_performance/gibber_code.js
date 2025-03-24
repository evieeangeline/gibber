Clock.bpm = 120

Theory.degree.seq( ['i','-iv','-V'], [8,4,4] )

delay = Delay( '1/6' ).bus()
verb = Reverb('space').bus()


glitch = Freesound[5]({ query:'glitch', max:.5 })
  .connect( verb, .05 )
  .spread(1) // pan voices full stereo

glitch.pickplay.seq(
  Rndi(0,14),
  Euclid(9,16)
)
  
kick = Kick('deep')
  .connect( verb, .5 )
  .trigger.seq( 1, 4 )


h = Hat().connect( verb, .15 )
h.trigger.tidal( '<.5 .35*3 [.5 .25] [.75 .25 .5 .25]>' )
  
perc = Synth[4]( 'square.perc' ).connect( verb, .35 )
  .spread(1)
  .note.seq( sine(2,7) , Hex(0x8036)/* 1000000000110110 */  )
  .note.seq( sine(2.25, 4, 7 ) , Hex(0x4541)/* 0100010101000001 */ , 1 )
  .loudnessV.seq( sine(1.5, .5, .65 )  )


bass.note.tidal( '0 0 0 0 4 6 0 ~ 0 ~ 7 -7 ~ 0 -7 0' )
bass.cutoff = gen( 0.5 + cycle(0.07) * 0.45 ) 

pad = Synth[4]('rhodes', { decay:8, gain:.15 })
pad.fx[0].connect( Out, .125)
pad.fx[0].connect( verb, 1 )
pad.chord.seq([[0,2,4,6], [1,2,4,7]], 4 )
 

