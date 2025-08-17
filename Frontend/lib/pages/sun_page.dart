import 'dart:convert';
import 'package:flutter/material.dart';
import '../widgets/app_top_bar.dart';
import '../realtime_service.dart' show realtime;

class SunPage extends StatefulWidget {
  const SunPage({super.key});
  @override
  State<SunPage> createState() => _SunPageState();
}

class _SunPageState extends State<SunPage> {
  static const bool _useFixedCity = true;
  static const String _fixedCity = 'MADRID';

  @override
  void initState() {
    super.initState();
    realtime.connect(); 

  Map<String, dynamic> _asMap(dynamic v) {
    if (v is Map) return Map<String, dynamic>.from(v);
    if (v is String) {
      try {
        final d = jsonDecode(v);
        if (d is Map) return Map<String, dynamic>.from(d);
      } catch (_) {}
    }
    return const <String, dynamic>{};
  }

  num? _num(dynamic v) {
    if (v is num) return v;
    if (v is String) return num.tryParse(v);
    return null;
  }

  String? _str(dynamic v) => v == null ? null : v.toString();

  String _aqiLabel(num? aqi) {
    if (aqi == null) return '—';
    final x = aqi.toDouble();
    if (x <= 50) return 'Good';
    if (x <= 100) return 'Moderate';
    if (x <= 150) return 'Unhealthy (SG)';
    if (x <= 200) return 'Unhealthy';
    if (x <= 300) return 'Very unhealthy';
    return 'Hazardous';
  }

  String _weekdayAbbr(DateTime d) {
    const names = ['Mon', 'Tue', 'Wed', 'Thu', 'Fri', 'Sat', 'Sun'];
    return names[(d.weekday - 1) % 7]; 
  }

  double _tileHeight(BuildContext context) {
    final scale = MediaQuery.of(context).textScaleFactor.clamp(1.0, 1.4);
    return 116.0 * scale;
  }

  String _buildEcoTips(Map<String, dynamic> msg) {
    final tip = _str(msg['eco_tips'])?.trim();
    if (tip != null && tip.isNotEmpty) return tip;

    final reco = _asMap(msg['reco']);
    final actions = (reco['actions'] is List) ? List.from(reco['actions']) : const [];
    final filtered = actions
        .where((e) => e != null && e.toString().trim().isNotEmpty)
        .map((e) => '• ${e.toString().trim()}')
        .toList();
    if (filtered.isNotEmpty) return filtered.take(4).join('\n');

    return 'Tips will adapt automatically from the live context (UV, heat, humidity).';
  }

  @override
  Widget build(BuildContext context) {
    final theme = Theme.of(context);
    final primary = theme.colorScheme.primary;

    final today = DateTime.now();
    final dayLine = '${_weekdayAbbr(today)} ${today.day}';

    return Scaffold(
      backgroundColor: theme.scaffoldBackgroundColor,
      appBar: const PreferredSize(
        preferredSize: Size.fromHeight(70),
        child: AppTopBar(),
      ),
      body: SafeArea(
        child: Padding(
          padding: const EdgeInsets.all(16),
          child: StreamBuilder(
            stream: realtime.stream,
            builder: (context, snap) {
              final Map<String, dynamic> msg = _asMap(snap.data);
              final Map<String, dynamic> ctx = _asMap(msg['context']);
              final Map<String, dynamic> tel = _asMap(msg['telemetry']);

              final String city = _useFixedCity
                  ? _fixedCity
                  : (_str(ctx['city'])?.toUpperCase() ?? _fixedCity);

              final double? outdoorTemp = _num(
                ctx['temperature'] ??
                    ctx['temp'] ??
                    ctx['temp_c'] ??
                    ctx['t'] ??
                    ctx['t2m'] ??
                    ctx['ambient_temp'],
              )?.toDouble();

              final double? humidity = _num(ctx['humidity'] ?? ctx['humidity_pct'])?.toDouble();
              final double? pressure = _num(ctx['pressure'] ?? ctx['pressure_hpa'])?.toDouble();

              final double? windKmH = _num(
                ctx['wind_speed'] ?? ctx['wind_kmh'] ?? ctx['windspeed_10m'],
              )?.toDouble();

              final double? uv = _num(
                ctx['uv_index'] ?? ctx['uv'] ?? ctx['uvi'],
              )?.toDouble();

              final int? aqi = _num(ctx['aqi'] ?? ctx['pm25_aqi'])?.toInt();
              final String airQuality =
                  _str(ctx['air_quality']) ?? _aqiLabel(aqi);

              final double? roomTemp = _num(
                tel['t_amb'] ?? tel['temp_amb'] ?? tel['ambient_temp'],
              )?.toDouble();

              final int? co2Ppm = _num(
                tel['co2'] ??
                    (tel['env'] is Map ? (tel['env'] as Map)['co2'] : null) ??
                    (tel['m'] is Map ? (tel['m'] as Map)['co2'] : null),
              )?.toInt();

              final int? lux = _num(
                tel['lx'] ??
                    (tel['env'] is Map ? (tel['env'] as Map)['lx'] : null) ??
                    (tel['m'] is Map ? (tel['m'] as Map)['lx'] : null),
              )?.toInt();

              final tileHeight = _tileHeight(context);
              final tipsText = _buildEcoTips(msg);

              return ListView(
                children: [
                  Padding(
                    padding: const EdgeInsets.only(top: 6, bottom: 10),
                    child: Text(
                      'Current location',
                      textAlign: TextAlign.center,
                      style: theme.textTheme.bodyMedium?.copyWith(
                        color: primary,
                        fontWeight: FontWeight.w700,
                      ),
                    ),
                  ),

                  _CityCard(
                    city: city,
                    dayLine: dayLine,
                    temperature: outdoorTemp == null
                        ? '--'
                        : '${outdoorTemp.toStringAsFixed(0)}°',
                  ),
                  const SizedBox(height: 16),

                  _ActionsCard(
                    title: 'Daily actions for a greener city',
                    body: tipsText,
                  ),
                  const SizedBox(height: 16),

                  GridView(
                    physics: const NeverScrollableScrollPhysics(),
                    shrinkWrap: true,
                    gridDelegate: SliverGridDelegateWithFixedCrossAxisCount(
                      crossAxisCount: 2,
                      mainAxisExtent: tileHeight,
                      crossAxisSpacing: 12,
                      mainAxisSpacing: 12,
                    ),
                    children: [
                      _MetricTile(
                        title: 'Ambient temperature',
                        value: outdoorTemp == null
                            ? '--'
                            : '${outdoorTemp.toStringAsFixed(1)}°C',
                      ),
                      _MetricTile(
                        title: 'Humidity',
                        value: humidity == null
                            ? '--'
                            : '${humidity.toStringAsFixed(0)}%',
                      ),
                      _MetricTile(
                        title: 'Pressure',
                        value: pressure == null
                            ? '--'
                            : '${pressure.toStringAsFixed(0)} hPa',
                      ),
                      _MetricTile(
                        title: 'CO₂',
                        value: co2Ppm == null ? '--' : '$co2Ppm ppm',
                      ),
                      _MetricTile(
                        title: 'Wind speed',
                        value: windKmH == null
                            ? '--'
                            : '${windKmH.toStringAsFixed(0)} km/h',
                      ),
                      _MetricTile(title: 'Air quality', value: airQuality),
                      _MetricTile(
                        title: 'Ambient light',
                        value: lux == null ? '--' : '$lux lx',
                      ),
                      _MetricTile(
                        title: 'UV index',
                        value: uv == null ? '--' : uv.toStringAsFixed(1),
                      ),
                    ],
                  ),

                  const SizedBox(height: 24),
                ],
              );
            },
          ),
        ),
      ),
    );
  }
}


class _CityCard extends StatelessWidget {
  const _CityCard({
    required this.city,
    required this.dayLine,
    required this.temperature,
  });

  final String city;
  final String dayLine; 
  final String temperature; 

  @override
  Widget build(BuildContext context) {
    final theme = Theme.of(context);
    final cs = theme.colorScheme;
    final isDark = theme.brightness == Brightness.dark;

    final Color bg = isDark ? const Color(0xFF232426) : const Color(0xFFFFF1DE);
    final Color border = isDark ? Colors.white10 : const Color(0xFFFFD8A6);

    return Container(
      padding: const EdgeInsets.symmetric(horizontal: 22, vertical: 26),
      decoration: BoxDecoration(
        color: bg,
        borderRadius: BorderRadius.circular(26),
        border: Border.all(color: border),
      ),
      child: Column(
        mainAxisSize: MainAxisSize.min,
        children: [
          Text(
            city,
            textAlign: TextAlign.center,
            style: theme.textTheme.titleLarge?.copyWith(
              color: cs.primary,
              fontWeight: FontWeight.w900,
              letterSpacing: .8,
            ),
          ),
          const SizedBox(height: 4),

          Text(
            dayLine,
            style: theme.textTheme.bodySmall?.copyWith(
              color: cs.primary.withOpacity(.85),
              fontWeight: FontWeight.w600,
            ),
          ),
          const SizedBox(height: 12),

          Icon(Icons.wb_sunny_rounded, size: 56, color: cs.primary),
          const SizedBox(height: 14),

          FittedBox(
            fit: BoxFit.scaleDown,
            child: Text(
              temperature,
              style: theme.textTheme.displayMedium?.copyWith(
                color: cs.primary,
                fontWeight: FontWeight.w900,
                letterSpacing: .6,
              ),
            ),
          ),
        ],
      ),
    );
  }
}

class _ActionsCard extends StatelessWidget {
  const _ActionsCard({required this.title, required this.body});
  final String title;
  final String body;

  @override
  Widget build(BuildContext context) {
    final theme = Theme.of(context);
    final isDark = theme.brightness == Brightness.dark;
    final outline = isDark ? Colors.white12 : const Color(0xFFFFD8A6);

    return Container(
      padding: const EdgeInsets.all(16),
      decoration: BoxDecoration(
        color: theme.cardColor,
        borderRadius: BorderRadius.circular(22),
        border: Border.all(color: outline),
      ),
      child: Row(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Icon(Icons.lightbulb, color: theme.colorScheme.primary),
          const SizedBox(width: 12),
          Expanded(
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                Text(
                  title,
                  style: theme.textTheme.titleMedium?.copyWith(
                    fontWeight: FontWeight.w700,
                  ),
                ),
                const SizedBox(height: 6),
                Text(
                  body,
                  style: theme.textTheme.bodyMedium?.copyWith(
                    color: theme.textTheme.bodyMedium?.color?.withOpacity(.85),
                  ),
                ),
              ],
            ),
          ),
        ],
      ),
    );
  }
}

class _MetricTile extends StatelessWidget {
  const _MetricTile({required this.title, required this.value});
  final String title;
  final String value;

  @override
  Widget build(BuildContext context) {
    final theme = Theme.of(context);
    final isDark = theme.brightness == Brightness.dark;
    final outline = isDark ? Colors.white12 : const Color(0xFFFFD8A6);

    return Container(
      padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 14),
      decoration: BoxDecoration(
        color: theme.cardColor,
        borderRadius: BorderRadius.circular(20),
        border: Border.all(color: outline),
      ),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Text(
            title,
            maxLines: 2,
            overflow: TextOverflow.ellipsis,
            style: theme.textTheme.bodyMedium?.copyWith(
              color: theme.colorScheme.primary,
              fontWeight: FontWeight.w700,
            ),
          ),
          const Spacer(),
          Text(
            value,
            maxLines: 1,
            overflow: TextOverflow.ellipsis,
            style: theme.textTheme.titleLarge?.copyWith(
              fontWeight: FontWeight.w900,
              letterSpacing: .2,
            ),
          ),
        ],
      ),
    );
  }
}