package frc.chargers.utils;

import edu.wpi.first.wpilibj.Alert;

public class CustomAlerts {
	public static class CountableAlert extends Alert {
		private int count = 0;
		private final String text;
		
		public CountableAlert(String text, AlertType type) {
			super(text + "(1)", type);
			this.text = text;
		}
		
		public CountableAlert(String group, String text, AlertType type) {
			super(group, text + "(1)", type);
			this.text = text;
		}
		
		public void addOccurrence() {
			count++;
			setText(text + "(" + count + ")");
			set(true);
		}
	}
}
