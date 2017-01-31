<slider label='{ opts.label }'>
    <div class='sliderInnerContainer'>
        <div>{ opts.label } = <span>{ mantissa }</span> × 10<sup>{ exponent }</sup></div>
        <div>
            <button>Center</button>
            <button>Reset</button>
        </div>
    </div>

    <div class='sliderInnerContainer'>
        <div class='bound left'><span class='times' show={ centered }>× </span>10<sup>1</sup></div>
        <input type='range' />
        <div class='bound right'><span class='times' show={ centered }>× </span>10<sup>5</sup></div>
    </div>

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

