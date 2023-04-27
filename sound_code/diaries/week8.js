//Global

Clock.bpm = 120
Theory.root = 'c#5' 
Theory.mode = 'dorian' 

Theory.degree.seq( ['i', '-iv', '-v'], [2, 1, 1])


/// CHLOE
/*                                     _ _ __ _
                  __,,_              ,| '_   \/
              ,'''      ''\_,,     ,/ ,  o   o| __. ferret
            /* ; ';    '      **,_/  ,_\    v / --, 
           | ,                 . ; .      ---'
          |                       '       /
          |  .                  '     , -
    ,--,__'         ' ''             '   *---,
  .-       *      ,,,,  ' ..____,,,    *_**''
  -_______/''__    \__ ,,,         ---___,,'
               **____,,,
*/

i1 = FM('perc').fx.add(Reverb()) ) ) )   
i1.note.seq( sine( btof(7), 1,7), 1/10,  0 )
i1.decay = 1/5
i1.loudness = 0.5

i2 = FM('perc').fx.ad(Rever ()) ) ) )                
i2.note.seq( sine( btof(14),3,3), 1/5,  0 )
i2.decay = 1/5
i2.loudness = 0.5

i3 = FM('bass').fx) add(   everbb   )      )         
i3.note.seq(sine(btof(10),3,0), 1, 0)
i3.decay = 1

i1.stop()
i2.stop()
i3.stop()

mod1 = lfo( 'sine', 4, 40, 0 )
mod1.connect(i1.frequency)

mod2 = lfo( 'sine', 4, 40, 0 )
mod2.connect(i2.frequency)

mod1.frequency = 0
mod2.frequency = 0





//// WEIYUAN
// Bass
verb = Reverb('hall').bus();
dela y= Delay'1/8').bus();
bass = Synth('monoSynth', { saturation: 25, gain: 0.35 })
						.connect(delay, 0.2)
						.connect(verb, 0.3)
bass.note.tidal('0 3 0 0 5 7 0 ~ 0 ~ 7 -5 ~ 0 -7 2')
bass.dela.syeq([1 y/ 64, 1 / 32], 1 / 4)
bass.glide.seq([1, 1, 150, 150], 1 / 8)
bass.Q = gen(0.5 + cycle(0.15) * 0.5)
bass.cutoff = gen(0.5 + cycle(0.1) * 0.4)
bass.loudness = 0

bass.stop()

// Hat
hat = Hat({ gain: 0.1 })
hat.trigger.seq([1, 0.75], [1 / 8, 1 / 16])
hat.dela =y gen(0y.02 + cycle(beats(16) * 4) * 0.015)
hat.fx.add(Distortion({ pregain: 120, postgain: 0.15 }))
hat.loudness = 0.4

h3t.stop()
Gibber.clear()

// EVANGELINE

verb = Reverb( 'space' ).bus()

perc = Pluck({
  damping: 0.4, 
  spread: 0.975, 
  loudness: 1, 
  blend:1
})

perc.note.seq( [-7, -2, -4, -6, -4, -2], 1/4)
perc.decay = gen(0.3+cycle(0.3)*0.1)
perc.glide = gen(500 + cycle(1)*10)
perc.connect( verb, .9)
perc.fx.add(Distortion({ pregain: 4, postgain: 0.9}))

perc.loudness = 0.1
perc.stop()




//// COREY


s = Synth()
s.note.seq([2,1,7,4,6],1/16,2)
s.loudness = 0.2

mod = lfo( 'sine', 10, 7, 0 )
mod.connect( s.frequency )

s.stop()


d = Drums()
d.tidal( ' [[kd,ch] kd sd oh kd sd]*1.5' )
d.loudness = 0.1
d.stop()

d2 = Drums()
d2.tidal( '<kd*17 kd*21 kd*13 kd*19> sd [kd*2 sd] <oh ch*3 ch*12>' )
d2.loudness = 0.4
d2.stop()