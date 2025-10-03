using Toybox.WatchUi as WatchUi;

class HelloWorld extends WatchUi.WatchFace {
    function onUpdate(dc) {
        dc.clear();
        dc.drawText(dc.getWidth() / 2, dc.getHeight() / 2,
            WatchUi.FONT_XLARGE, "Hello, World!", WatchUi.TEXT_JUSTIFY_CENTER);
    }
}
