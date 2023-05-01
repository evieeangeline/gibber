use( 'hydra' ).then( init => init() )

Clock.bpm = 50
Theory.root = 'c'
Theory.mode = 'aeolian'
 
verb  = Reverb( 'space' ).bus() 
delay = Delay( '1/6' ).bus()

Theory.degree.seq( ['i','iv','VI'], [1/3,1/2,1/9] )





////// Evangelion (rawr)
drumsA = Drums()
 
drumsA.tidal('kd [kd, sd] kd [kd, sd]')
drumsA.connect( verb, 0.5)
drumsA.stop()

// high hat
hat = Hat({ gain:.075 })
hat.trigger.seq( [1,.5, 1,.5, 1,.5, 1,.5,.5,.5,.5], [1/8, 1/16,1/8, 1/16, 1/8, 1/16, 1/8, 1/16, 1/16, 1/32, 1/32, 1/32, 1/32 ] )
hat.decay = gen( .02 + cycle( beats(16) * 4 ) * .0125 )
hat.fx.add( Distortion({ pregain:100, postgain:.1 }) )

hat.stop()

//chords
padA = Synth[4]('rhodes', {octave:6, decay:8, gain:0.4 })
padA.fx[0].connect( verb, 1 )
padA.chord.seq([[21,23,25,27]], 1)

padA.stop()

// glitchey boy
glitch = Freesound[5]({ query:'glitch', max:.5 })
  .connect( verb, .05 )
  .spread(1) // pan voices full stereo

glitch.pickplay.seq(
  Rndi(0,14)/* 11 */,
  Euclid(9,16)/* 1011010101101010 */ 
)

glitch.stop()


// CROAR (wow)

s = FM()
s.fx[0].connect(verb1)
s.note.seq([30, 26, 28,23, 29],1/34,0)
s.loudness = 0.1
s.stop()






// weiy(chill)
bass = Synth('monoSynth', { saturation: 25, gain: 0.35 })
						.connect(delay, 0.2)
						.connect(verb, 0.3)
bass.note.tidal('0 1 0 0 1 7 0 ~ 0 ~ 7 -5 ~ 0 -7 2')
bass.delay.seq([1 / 64, 1 / 32], 1 / 4)
bass.glide.seq([1, 1, 150, 150], 1 / 8)
bass.Q = gen(0.5 + cycle(0.15) * 0.5)
bass.cutoff = gen(0.5 + cycle(0.1) * 0.4)
bass.loudness = 0.5

bass.stop()

//Y
verb1 = Reverb( 'space' ).bus() 
delay = Delay( '1/9' ).bus().connect( verb1, .1 )
syn = PolySynth('bleep.dry').connect( verb1, .8 )
syn.note.tidal( '<3 6> [7 4] [1 2] <3 6>' )
syn.pan.tidal( '0 1 0.5 1' )
syn.loudness=0.3

syn.stop()



//Graphics
osc(40,5,20).kaleid( syn.out(5,1.012) ).rotate(0, 0.1)
.scrollY(0, 0.01).scrollX(0, 0.01).blend(o0, 0.3)
.modulateHue(o0, 4).saturate(1.21).colorama(0.003)
.contrast(1.01).scale(1.25).out(o0)


osc(40,5,20).kaleid( bass.out(5,1.012) ).kaleid( syn.out(5,1.012) ).rotate(0, 0.1)
.scrollY(0, 0.01).scrollX(0, 0.01).blend(o0, 0.3)
.modulateHue(o0, 4).saturate(1.21).colorama(0.003)
.contrast(1.01).scale(1.25).out(o0)

osc(40,5,20).kaleid( hat.out(5,1.012) ).rotate(0, 0.1)
.scrollY(0, 0.01).scrollX(0, 0.01).blend(o0, 0.3)
.modulateHue(o0, 4).saturate(1.21).colorama(0.003)
.contrast(1.01).scale(1.25).out(o0)
hush()