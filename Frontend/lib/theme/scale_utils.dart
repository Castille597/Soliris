import 'package:flutter/widgets.dart';

extension AppScale on num {
  double sx(BuildContext context) =>
      this * MediaQuery.textScaleFactorOf(context).clamp(0.9, 1.6);
}
