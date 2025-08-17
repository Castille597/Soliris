import 'dart:convert';
import 'package:flutter/material.dart';

import '../realtime_service.dart' show realtime;
import '../alert_center.dart';

import '../widgets/app_top_bar.dart';
import '../widgets/greeting_header.dart';
import '../widgets/mood_selector.dart';
import '../widgets/day_selector.dart';
import '../widgets/recommendation_card.dart';

class DashboardPage extends StatefulWidget {
  const DashboardPage({super.key});

  @override
  State<DashboardPage> createState() => _DashboardPageState();
}

class _DashboardPageState extends State<DashboardPage> {
  @override
  void initState() {
    super.initState();
    realtime.connect();
    AlertCenter.instance.bindTo(realtime.stream);
  }

  Map<String, dynamic> _toMap(dynamic v) {
    if (v is Map) {
      return v.map((k, val) => MapEntry(k.toString(), val));
    }
    if (v is String) {
      try {
        final d = jsonDecode(v);
        if (d is Map) {
          return d.map((k, val) => MapEntry(k.toString(), val));
        }
        return <String, dynamic>{'raw': d};
      } catch (_) {
        return <String, dynamic>{'raw': v};
      }
    }
    if (v == null) return const <String, dynamic>{};
    return <String, dynamic>{'raw': v.toString()};
  }

  @override
  Widget build(BuildContext context) {
    final theme = Theme.of(context);

    return Scaffold(
      backgroundColor: theme.scaffoldBackgroundColor,
      body: SafeArea(
        child: SingleChildScrollView(
          padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 20),
          child: Column(
            crossAxisAlignment: CrossAxisAlignment.start,
            children: [
              const AppTopBar(),
              const SizedBox(height: 20),
              const GreetingHeader(),
              const SizedBox(height: 20),
              const MoodSelector(),
              const SizedBox(height: 20),
              const DaySelector(),
              const SizedBox(height: 30),

              StreamBuilder(
                stream: realtime.stream,
                builder: (context, snap) {
                  final Map<String, dynamic> msg = _toMap(snap.data);

                  return RecommendationCard(
                    msg: msg,
                    title: 'Wellness recommendations',
                    isLive: true,
                  );
                },
              ),

              const SizedBox(height: 30),
            ],
          ),
        ),
      ),
    );
  }
}
