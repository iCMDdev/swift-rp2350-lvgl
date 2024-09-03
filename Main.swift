
@main
struct Main {
  static func main() {
    stdio_init_all();
    show_demo()
    
    while true {
      lv_timer_handler()
    }
  }
}
