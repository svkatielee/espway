<slider my-value={ value }>
    <div class='sliderInnerContainer'>
        <div>
            <span class='bold'>{ opts.label }</span> =
            <span>{ mantissa }</span> × 10<sup>{ exponent }</sup>
        </div>
        <div>
            <button onclick={ centerClick }>Center</button>
            <button onclick={ resetClick }>Reset</button>
        </div>
    </div>

    <div class='sliderInnerContainer'>
        <div class='bound left'>
            <span class='times' show={ centered }>× </span>10<sup>{ boundMin }</sup>
        </div>
        <input type='range' ref='slider' oninput={ sliderChange }
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
        let value = Math.pow(10, this.refs.slider.value)
        if (this.centered) {
            value *= this.centerValue
        }
        this.value = value
    }

    refreshValue() {
        let sliderValue = log10(this.value)
        if (this.centered) {
            sliderValue -= log10(this.centerValue)
        }
        this.refs.slider.value = sliderValue
        let exponent = Math.floor(log10(this.value))
        let mantissa = this.value / Math.pow(10, exponent)
        this.mantissa = mantissa.toFixed(1)
        this.exponent = exponent
    }

    resetSlider() {
        this.centered = false
        this.setBounds(this.opts.boundMin, this.opts.boundMax)
    }

    centerClick(e) {
        this.centered = true
        this.centerValue = this.value
        this.setBounds(-1, 1)
        this.refs.slider.value = 0
    }

    resetClick(e) {
        this.resetSlider()
    }

    this.resetSlider()

    this.on('mount', () => {
        this.value = this.opts.myValue
        this.update()
    })

    this.on('update', () => {
        this.refreshValue()
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
</slider>

