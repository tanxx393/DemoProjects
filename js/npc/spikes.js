import Sprite   from '../base/sprite'
import DataBus  from '../databus'

const SPIKE_IMG_SRC = 'images/iceBullet.png'
const SPIKE_WIDTH   = 20
const SPIKE_HEIGHT  = 30
const ENEMY_WIDTH   = 60
const ENEMY_HEIGHT  = 60

const __ = {
  speed: Symbol('speed')
}

let databus = new DataBus()

export default class Spike extends Sprite {
  constructor() {
    super(SPIKE_IMG_SRC, SPIKE_WIDTH, SPIKE_HEIGHT)
  }

  init(x, y, speed) {
    console.log("spike: ", x, y)

    this.x = x + ENEMY_WIDTH / 2 - SPIKE_WIDTH / 2
    this.y = y + ENEMY_HEIGHT / 2 + 10
  

    this[__.speed] = speed

    this.visible = true
  }
  

  // 每一帧更新子弹位置
  update() {
    // console.log("Here", this.x, this.y, this.visible)
    // this.y += this[__.speed]

    // // 超出屏幕外回收自身
    // if ( this.y > this.height )
    //   // this.visible = false
    //   databus.removeSpikes(this)
  }
}
