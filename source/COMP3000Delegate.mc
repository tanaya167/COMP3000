import Toybox.Lang;
import Toybox.WatchUi;

class COMP3000Delegate extends WatchUi.BehaviorDelegate {

    function initialize() {
        BehaviorDelegate.initialize();
    }

    function onMenu() as Boolean {
        WatchUi.pushView(new Rez.Menus.MainMenu(), new COMP3000MenuDelegate(), WatchUi.SLIDE_UP);
        return true;
    }

}