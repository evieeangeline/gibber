


// clave - Euclidien Demo
clave = Clave({ gain:.1 }).connect( verb, .25 )
  .trigger.seq( .5, e = Euclid(3,8)/* 10010010 */  )
clave.stop()

// Basic Drums beat
drumsA = Drums()
drumsA.fx.add( Distortion({ pregain:1.5, postgain:1 }) )
 
drumsA.tidal('kd [kd, sd] kd [kd, sd]')

drumsA.stop()

// high hat
hat = Hat({ gain:.075 })
hat.trigger.seq( [1,.5], [1/8, 1/16] )
hat.decay = gen( .02 + cycle( beats(16) * 4 ) * .0125 )
hat.fx.add( Distortion({ pregain:100, postgain:.1 }) )

hat.stop()