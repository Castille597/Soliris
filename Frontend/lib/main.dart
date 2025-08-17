import 'package:flutter/material.dart';
import 'theme/theme_controller.dart';
import 'pages/welcome_screen.dart';
import 'pages/home_navigator.dart';
import 'profile_store.dart';

import 'realtime_service.dart' show realtime;
import 'alert_center.dart';

const bool kDemoResetOnStart = true;

Future<void> main() async {
  WidgetsFlutterBinding.ensureInitialized();

  await AlertCenter.instance.init();

  if (kDemoResetOnStart) {
    await ProfileStore.instance.clear();
  } else {
    await ProfileStore.instance.load();
  }

  runApp(const SolirisApp());
}

class SolirisApp extends StatefulWidget {
  const SolirisApp({super.key});
  @override
  State<SolirisApp> createState() => _SolirisAppState();
}

class _SolirisAppState extends State<SolirisApp> {
  @override
  void initState() {
    super.initState();
    realtime.connect();
    AlertCenter.instance.bindTo(realtime.stream);
  }

  @override
  void dispose() {
    AlertCenter.instance.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return ValueListenableBuilder<ThemeMode>(
      valueListenable: ThemeController.instance.mode,
      builder: (_, mode, __) {
        return ValueListenableBuilder<double>(
          valueListenable: ThemeController.instance.textScale,
          builder: (_, scaleRaw, __) {
            final double scale = scaleRaw.clamp(0.9, 1.6);

            return MaterialApp(
              debugShowCheckedModeBanner: false,
              title: 'Soliris',

              theme: ThemeController.instance.light,
              darkTheme: ThemeController.instance.dark,
              themeMode: mode,

              builder: (context, child) {
                final mq = MediaQuery.of(
                  context,
                ).copyWith(textScaleFactor: scale);

                final base = Theme.of(context);
                final appBar = base.appBarTheme;

                final newAppBar = appBar.copyWith(
                  toolbarHeight: (appBar.toolbarHeight ?? 56) * scale,
                  titleTextStyle:
                      (appBar.titleTextStyle ??
                              base.textTheme.titleLarge ??
                              const TextStyle(
                                fontSize: 20,
                                fontWeight: FontWeight.bold,
                              ))
                          .copyWith(
                            fontSize:
                                ((appBar.titleTextStyle?.fontSize) ??
                                    base.textTheme.titleLarge?.fontSize ??
                                    20) *
                                scale,
                          ),
                  iconTheme: (appBar.iconTheme ?? base.iconTheme).copyWith(
                    size:
                        ((appBar.iconTheme?.size) ??
                            base.iconTheme.size ??
                            24) *
                        scale,
                  ),
                );

                final themed = base.copyWith(
                  appBarTheme: newAppBar,
                  iconTheme: base.iconTheme.copyWith(
                    size: (base.iconTheme.size ?? 24) * scale,
                  ),
                );

                return MediaQuery(
                  data: mq,
                  child: Theme(data: themed, child: child!),
                );
              },

              home: ValueListenableBuilder(
                valueListenable: ProfileStore.instance.profile,
                builder: (context, profile, _) {
                  return profile == null
                      ? const WelcomeScreen()
                      : const HomeNavigator();
                },
              ),
            );
          },
        );
      },
    );
  }
}
