# はじめに

**Squid-N** は、日本の建築構造計算一貫プログラムです。
Rust で実装されています。

このサイトは、Squid-N を利用する方に向けたドキュメントです。

- **[アーキテクチャ](./architecture.md)**：15 クレートから成る階層型構成の全体像
- **[MCP サーバ](./mcp_server.md)**：AI エージェントからモデル照会・解析実行・結果取得を行う
  MCP（Model Context Protocol）サーバのビルド・起動・ツール一覧
- **[計算根拠（理論・出典）](./calc_basis/README.md)**：各計算が「何という基準・法令の、何という式で」
  算定されているかを、告示・施行令の条／式番号と実装位置まで対応づけた根拠ドキュメント。
  荷重・材料・断面性能・構造解析・一次設計・二次設計・部材終局耐力・限界耐力計算・免震制振の
  各章を、算定項目ごとのページに分けて収録しています

## 開発者向け資料

設計仕様（specs）・検証記録（V&V）・開発運用ドキュメントは開発者向けのため本サイトには含めていません。
リポジトリの以下を参照してください。

- [specs/](https://github.com/hrntsm/squid-n/tree/main/specs)：フェーズ単位の実装仕様書と原典（法令・規準）照合リスト
- [v_and_v/](https://github.com/hrntsm/squid-n/tree/main/v_and_v)：各要素・各設計式の Verification & Validation レポート
- [dev_docs/](https://github.com/hrntsm/squid-n/tree/main/dev_docs)：ロードマップ、申し送り、UI 関連ドキュメント

## リポジトリ

ソースコードは [github.com/hrntsm/squid-n](https://github.com/hrntsm/squid-n) にあります。

ビルド・テスト・静的解析の手順はリポジトリの `README.md` を参照してください。
