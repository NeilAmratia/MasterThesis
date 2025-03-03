#pragma once
#include <QAbstractListModel>
#include <QVector>
#include <QVariantMap>

class MessageModel : public QAbstractListModel {
    Q_OBJECT

public:
    enum Roles {
        MessageRole = Qt::UserRole + 1,
        TypeRole,
        NumberRole,
        TimestampRole
    };

    MessageModel(QObject* parent = nullptr);
    int rowCount(const QModelIndex & = QModelIndex()) const override;
    QVariant data(const QModelIndex &index, int role = Qt::DisplayRole) const override;
    QHash<int, QByteArray> roleNames() const override;

public slots:
    void appendMessage(const QVariantMap &message);

private:
    QVector<QVariantMap> m_messages;
    int m_messageCounter;
};