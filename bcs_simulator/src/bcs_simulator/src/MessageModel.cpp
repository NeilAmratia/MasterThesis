#include "MessageModel.hpp"

MessageModel::MessageModel(QObject* parent) : QAbstractListModel(parent), m_messageCounter(0) {}

int MessageModel::rowCount(const QModelIndex &) const {
    return m_messages.size();
}

QVariant MessageModel::data(const QModelIndex &index, int role) const {
    if (!index.isValid() || index.row() >= m_messages.size())
        return QVariant();

    const auto &message = m_messages[index.row()];
    switch(role) {
        case MessageRole:
            return message["message"];
        case TypeRole:
            return message["type"];
        case NumberRole:
            return message["number"];
        case TimestampRole:
            return message["timestamp"];
    }
    return QVariant();
}

QHash<int, QByteArray> MessageModel::roleNames() const {
    QHash<int, QByteArray> roles;
    roles[MessageRole] = "message";
    roles[TypeRole] = "type";
    roles[NumberRole] = "number";
    roles[TimestampRole] = "timestamp";
    return roles;
}

void MessageModel::appendMessage(const QVariantMap &message) {
    beginInsertRows(QModelIndex(), m_messages.size(), m_messages.size());
    m_messages.append(message);
    endInsertRows();
}