module.exports = function() {
  const select = document.querySelector( 'select' )
  const files = [
    {
      name:'demos',
      options:[
        ['intro', 'newintro.js'],
        ['pick your sample', 'picksomesamples.js'],
        ['acid', 'acid.js'],
        ['moody', 'intro.js'],
        ['geometry melds', 'meld.js']
      ]
    },

    {
      name:'general tutorials',
      options:[['1. running/stopping code', 'intro.tutorial.js'],
        ['2. creating objects', 'creating.objects.js'],
        ['3. basic sequencing', 'sequencing.js'],
        ['4. patterns', 'pattern.js'],
        ['5. audiovisual mappings', 'mapping.js'],
        ['6. tidalcycles', 'tidal.js' ],
        ['7. modulation', 'modulation.js' ]
      ]
    },

    {
      name:'music tutorials',
      options:[
        ['scales/tunings', 'scales.tunings.js'],
        ['arpeggios and signals', 'arp.js' ], 
        ['polyphony', 'polyphony.js' ], 
        ['freesound', 'freesound.js' ], 
        ['samplers', 'sampler.js' ],
        ['step sequencing', 'steps.js' ],  
      ]
    }, 

    {
      name:'sound design tutorials',
      options:[
        ['oscillators', 'sounddesign_oscillators.js'],
        ['envelopes', 'sounddesign_envelopes.js'],
        ['filters', 'sounddesign_filters.js'],
        ['effects and busses', 'effects.js'],
        ['understanding the monosynth', 'monosynth.js'],
        ['creating synths', 'make.js' ]
      ]
    }, 

    {
      name:'graphics tutorials',
      options:[
        ['intro to constructive solid geometry', 'graphics.intro.js' ],  
        ['lighting and materials', 'graphics.lighting.js' ], 
        ['textures', 'texture.js' ],
        ['hydra', 'hydra.js' ]
      ]
    }
  ]

  for( let cat of files ) {
    const group = document.createElement('optgroup')
    group.setAttribute('label', cat.name )
    for( let file of cat.options ) {
      const opt = document.createElement('option')
      opt.innerText = file[0]
      opt.setAttribute( 'file', file[1] )
      group.appendChild( opt )
    }
    select.appendChild( group )
  }

  select.onchange = function( e ) {
    loadexample( select[ select.selectedIndex ].getAttribute( 'file' ) )
  }

  const loadexample = function( filename ) {
    const  req = new XMLHttpRequest()
    req.open( 'GET', './examples/'+filename, true )
    req.onload = function() {
      const js = req.responseText
      window.Environment.editor.setValue( js )
    }

    req.send()
  }
}