<my-slider label={ opts.label }>
    <div class='sliderInnerContainer'>
        <div><span class='bold'>{ opts.label }</span> = <span>{ mantissa }</span> × 10<sup>{ exponent }</sup></div>
        <div>
            <button onclick={ centerClick }>Center</button>
            <button onclick={ resetClick }>Reset</button>
        </div>
    </div>

    <div class='sliderInnerContainer'>
        <div class='bound left'>
            <span class='times' show={ centered }>× </span>10<sup>{ boundMin }</sup>
        </div>
        <input type='range' ref='slider' oninput={ sliderChange } value={ sliderValue }
            min={ boundMin } max={ boundMax } step={ step } />
        <div class='bound right'>
            <span class='times' show={ centered }>× </span>10<sup>{ boundMax }</sup>
        </div>
    </div>

    function log10(x) {
        return Math.log(x) * Math.LOG10E;
    }

    setBounds(boundMin, boundMax) {
        this.boundMin = boundMin
        this.boundMax = boundMax
        this.step = (boundMax - boundMin) / 100
    }

    sliderChange(e) {
        this.refreshValue()
    }

    refreshValue() {
        let val = this.refs.slider.value
        let expval = Math.pow(10, val)
        if (this.centered) {
            expval *= this.centerValue
        }
        let exponent = Math.floor(log10(expval))
        let mantissa = expval / Math.pow(10, exponent)
        this.mantissa = mantissa.toFixed(1)
        this.exponent = exponent
    }

    centerClick(e) {
        this.centered = true
        let val = this.refs.slider.value
        this.centerValue = Math.pow(10, val)
        this.setBounds(-1, 1)
        this.refs.slider.value = 0
    }

    resetClick(e) {
        this.centered = false
        this.setBounds(-2, 3)
    }

    this.centered = false
    this.centerValue = 0
    this.setBounds(-2, 3)

    this.on('mount', () => {
        this.refs.slider.value = this.boundMin
        this.refreshValue()
        this.update()
    })

    <style>
        .sliderInnerContainer > input {
            display: block;
            width: 100%;
            padding: 0;
            margin: 0;
        }

        .sliderInnerContainer {
            display: flex;
            justify-content: space-between;
            align-items: center;
            margin-top: 0.5em;
        }

        .bound {
            width: 5em;
        }

        .right {
            text-align: right;
        }

        button {
            border-radius: 0.5em;
            border-style: none;
            padding: 0.5em;
            background-color: #0bf;
            color: white;
            font-weight: bold;
        }

        my-slider {
            display: block;
            margin-bottom: 2em;
        }

        .bold {
            font-weight: bold;
        }
    </style>
</my-slider>

