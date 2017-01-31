<slider label={ opts.label }>
    <div class='sliderInnerContainer'>
        <div>{ opts.label } = <span>{ mantissa }</span> × 10<sup>{ exponent }</sup></div>
        <div>
            <button onclick={ centerclick }>Center</button>
            <button>Reset</button>
        </div>
    </div>

    <div class='sliderInnerContainer'>
        <div class='bound left'>
            <span class='times' show={ centered }>× </span>10<sup>{ boundMin }</sup>
        </div>
        <input ref='rangeslider' type='range' oninput={ sliderChange }
            min={ boundMin } max={ boundMax} step={ step }/>
        <div class='bound right'>
            <span class='times' show={ centered }>× </span>10<sup>{ boundMax }</sup>
        </div>
    </div>

    let refs = this.refs

    function resetSlider(boundMin, boundMax) {
        refs.rangeslider.min = boundMin
        refs.rangeslider.max = boundMax
        refs.rangeslider.step = (boundMax - boundMin) / 100
    }

    sliderChange(e) {
        let val = Math.pow(10, this.refs.rangeslider.value)
        let exponent = Math.floor(this.refs.rangeslider.value)
        let mantissa = val / Math.pow(10, exponent)
        this.mantissa = mantissa.toFixed(1)
        this.exponent = exponent
    }

    this.boundMin = -2
    this.boundMax = 3
    this.step = 5 / 100
    this.centered = false
    this.centervalue = 0

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
            width: 4.5em;
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
    </style>
</slider>

