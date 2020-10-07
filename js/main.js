import Player     from './player/index'
import Enemy      from './npc/enemy'
import Spike      from './npc/spikes'
import BackGround from './runtime/background'
import GameInfo   from './runtime/gameinfo'
import Music      from './runtime/music'
import DataBus    from './databus'

let ctx   = canvas.getContext('2d')
let databus = new DataBus()
const ENEMY_WIDTH   = 60
const ENEMY_HEIGHT  = 60

function rnd(start, end){
  return Math.floor(Math.random() * (end - start) + start)
}

/**
 * 游戏主函数
 */
export default class Main {
  constructor() {
    // 维护当前requestAnimationFrame的id
    this.aniId    = 0

    this.restart()
  }

  restart() {
    databus.reset()

    canvas.removeEventListener(
      'touchstart',
      this.touchHandler
    )

    this.bg       = new BackGround(ctx)
    this.player   = new Player(ctx)
    this.gameinfo = new GameInfo()
    this.music    = new Music()
    this.enemyActive = false
    this.initEnemy = true
    this.enemyX    = 0
    this.enemyY    = 0

    this.bindLoop     = this.loop.bind(this)
    this.hasEventBind = false

    // 清除上一局的动画
    window.cancelAnimationFrame(this.aniId);

    this.aniId = window.requestAnimationFrame(
      this.bindLoop,
      canvas
    )
  }

  /**
   * 随着帧数变化的敌机生成逻辑
   * 帧数取模定义成生成的频率
   */
  enemyGenerate() {
    if (( databus.frame % 100 === 0 && this.enemyActive == false) || this.initEnemy) {
      let enemy = databus.pool.getItemByClass('enemy', Enemy)
      this.enemyActive = true
      this.initEnemy = false
      databus.enemys = []
      this.enemyX = rnd(0, window.innerWidth - ENEMY_WIDTH)
      this.enemyY = window.innerHeight / 4 
      enemy.init(this.enemyX, this.enemyY, 6)
      databus.enemys.push(enemy)
    }
  }

  spikeShoot() {
    let spike = databus.pool.getItemByClass('spike', Spike)
    spike.init(this.enemyX, this.enemyY, 3)
    databus.spikes.push(spike)
  }

  // 全局碰撞检测
  collisionDetection() {
    let that = this
  
    databus.bullets.forEach((bullet) => {
      for ( let i = 0; i < 1; i++ ) {
        let enemy = databus.enemys[i]
        // console.log("Collision: ", enemy)
        if ( !enemy.isPlaying && enemy.isCollideWith(bullet)) {
          let health = enemy.updateCurrentHealth()
          bullet.visible = false
          if (health == 0) { 
            enemy.playAnimation()
            that.music.playExplosion()
            this.enemyActive = false
            this.initEnemy = true
            databus.score  += 1
            break
          }
        }
      }
    })
  
    databus.spikes.forEach((spike) => {
      for ( let i = 0; i < databus.spikes.length; i++ ) {
        let spike = databus.spikes[i]
        if (this.player.isCollideWith(spike)) {
          databus.gameOver = true
          break
        }
      }
    })

    for ( let i = 0, il = databus.enemys.length; i < il;i++ ) {
      let enemy = databus.enemys[i]

      if ( this.player.isCollideWith(enemy) ) {
        databus.gameOver = true
        break
      }
    }
  }

  // 游戏结束后的触摸事件处理逻辑
  touchEventHandler(e) {
     e.preventDefault()

    let x = e.touches[0].clientX
    let y = e.touches[0].clientY

    let area = this.gameinfo.btnArea

    if (   x >= area.startX
        && x <= area.endX
        && y >= area.startY
        && y <= area.endY  )
      this.restart()
  }

  /**
   * canvas重绘函数
   * 每一帧重新绘制所有的需要展示的元素
   */
  render() {
    ctx.clearRect(0, 0, canvas.width, canvas.height)

    this.bg.render(ctx)

    databus.bullets
          .concat(databus.enemys, databus.spikes)
          .forEach((item) => {
              item.drawToCanvas(ctx)
            })

    this.player.drawToCanvas(ctx)

    databus.animations.forEach((ani) => {
      if ( ani.isPlaying ) {
        ani.aniRender(ctx)
      }
    })

    this.gameinfo.renderGameScore(ctx, databus.score)

    // 游戏结束停止帧循环
    if ( databus.gameOver ) {
      this.gameinfo.renderGameOver(ctx, databus.score)

      if ( !this.hasEventBind ) {
        this.hasEventBind = true
        this.touchHandler = this.touchEventHandler.bind(this)
        canvas.addEventListener('touchstart', this.touchHandler)
      }
    }
  }

  // 游戏逻辑更新主函数
  update() {
    if ( databus.gameOver )
      return;

    // this.bg.update()

    databus.bullets
           .concat(databus.enemys, databus.spikes)
           .forEach((item) => {
              // console.log(item)
              item.update()
            })

    this.enemyGenerate()

    this.collisionDetection()

    if ( databus.frame % 100 === 0 ) {
      this.spikeShoot()
      // this.player.shoot()
      //this.player.shoot2()
      //this.player.shoot3()
      //this.music.playShoot()
    }
  }

  // 实现游戏帧循环
  loop() {
    databus.frame++

    this.update()
    this.render()

    this.aniId = window.requestAnimationFrame(
      this.bindLoop,
      canvas
    )
  }
}
